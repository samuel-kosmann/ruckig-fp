# ruckig_fp — Fixed-Point S-Curve Motion Library for RP2040

A single-axis, third-order (jerk-limited) S-curve motion profile library
written in pure C99 for the **Raspberry Pi Pico / RP2040** microcontroller.
Inspired by [pantor/ruckig](https://github.com/pantor/ruckig), but
redesigned for embedded use with **no floating-point arithmetic** in the
core path and **no dynamic memory allocation**.

---

## Introduction

Motion control on embedded systems requires smooth, jerk-limited velocity
profiles ("S-curves") to avoid mechanical resonance and driver faults.
Classic trapezoidal profiles (constant acceleration) produce infinite jerk
at segment boundaries; the 7-segment S-curve limits jerk to a maximum value,
producing smooth transitions that are kinder to both the mechanics and the
motor drive.

`ruckig_fp` targets the RP2040 (Cortex-M0+, no FPU), so all arithmetic in
the planner and quantisers uses **Q16.16 fixed-point integers** (`int32_t`).
This keeps every hot-path operation within 32-bit registers (with a few
deliberate 64-bit widening multiplies handled by `fp_mul` / `fp_div`).

---

## Repository Structure

```
ruckig_fp/
├── CMakeLists.txt              builds the static lib + optional example
├── README.md                   this file
├── include/
│   └── ruckig_fp/
│       ├── fixed_point.h       fp_t type, macros, inline math helpers
│       ├── profile.h           SCurveProfile struct and API
│       ├── ruckig_fp.h         trajectory planner public API
│       └── quantise.h          time-step and distance-step quantisers
└── src/
    ├── fixed_point.c           fp_sqrt (Newton-Raphson, no <math.h>)
    ├── profile.c               profile_integrate, profile_at_time
    ├── ruckig_fp.c             7-segment S-curve solver
    └── quantise.c              tsq_* and dsq_* iterator implementations
```

---

## Fixed-Point Representation

All physical quantities (position, velocity, acceleration, jerk, time) are
stored as `fp_t` = `int32_t` in **Q(32−F).F** format, where F = `FP_FRAC_BITS`
(default **16**).

| Parameter      | Value (FP_FRAC_BITS = 16) |
|----------------|---------------------------|
| Integer range  | −32 768 … +32 767         |
| Resolution     | 1 / 65 536 ≈ 1.5 × 10⁻⁵  |
| Scale factor   | 2¹⁶ = 65 536              |

### Choosing `FP_FRAC_BITS`

- **Higher F** → finer resolution but narrower integer range.
  Use F = 24 when positions are in µm and velocities < 127 µm/s.
- **Lower F** → wider integer range but coarser resolution.
  Use F = 8 when positions span millions of units (e.g. encoder counts
  on a high-resolution axis).
- The default **F = 16** works well for positions 0–32 767 units and
  velocities < 32 767 units/s.

Override by defining `FP_FRAC_BITS` before including `fixed_point.h` or
passing `-DFP_FRAC_BITS=24` on the compiler command line.

### Conversion Macros

```c
fp_t a = FP_FROM_INT(5);          // 5.0  → 327680  (compile-time)
fp_t b = FP_FROM_FLOAT(1.5f);     // 1.5  → 98304   (compile-time only)
int  i = FP_TO_INT(a);            // 327680 >> 16 = 5
float f = FP_TO_FLOAT(b);         // 98304 / 65536.0 = 1.5  (debug only)
```

> **Warning:** `FP_FROM_FLOAT` and `FP_TO_FLOAT` invoke the FPU (or a soft-
> float library).  Use them only at initialisation time or in debug builds,
> never in real-time ISR code.

---

## Mathematical Background

### 7-Segment UDDU S-Curve

A jerk-limited point-to-point move is decomposed into **7 segments**:

```
Segment:   1       2       3       4       5       6       7
Phase:     Accel   Accel   Accel   Coast   Decel   Decel   Decel
Jerk:      +J      0       −J      0       −J      0       +J
Accel:     rises   const   falls   0       falls   const   rises
```

The kinematic update equations for segment *i* of duration *dt* starting
from state (a₀, v₀, p₀):

```
a(dt) = a₀ + dt · j
v(dt) = v₀ + dt · a₀ + dt² · j / 2
p(dt) = p₀ + dt · v₀ + dt² · a₀/2 + dt³ · j/6
```

### Solving for Segment Durations

Given displacement Δp = pf − p0 and peak velocity V_peak ≤ v_max:

**Acceleration ramp** (v0 → V_peak):
- If V_peak − v0 ≤ A²/J: triangular ramp, t1 = t3 = √((V_peak−v0)/J), t2 = 0
- Else: trapezoidal ramp, t1 = t3 = A/J, t2 = (V_peak−v0)/A − A/J

**Deceleration ramp** (V_peak → vf) similarly gives t5, t6, t7.

**Coast phase:**
```
d_coast = Δp − d_accel − d_decel
t4      = d_coast / V_peak
```

If t4 < 0, V_peak is reduced by bisection until t4 ≥ 0.

---

## Quick-Start: DC Motor (Time-Step Quantiser)

```c
#include "ruckig_fp/ruckig_fp.h"
#include "ruckig_fp/quantise.h"

RuckigFpInput inp = {
    .p0    = FP_FROM_INT(0),
    .v0    = FP_FROM_INT(0),
    .a0    = FP_FROM_INT(0),
    .pf    = FP_FROM_INT(1000),   // move 1000 units
    .vf    = FP_FROM_INT(0),
    .af    = FP_FROM_INT(0),
    .v_max = FP_FROM_INT(200),    // 200 units/s peak speed
    .v_min = FP_FROM_INT(-200),
    .a_max = FP_FROM_INT(500),    // 500 units/s² peak accel
    .a_min = FP_FROM_INT(-500),
    .j_max = FP_FROM_INT(2000),   // 2000 units/s³ jerk limit
};

RuckigFpOutput out;
if (ruckig_fp_calculate(&inp, &out)) {
    TimeStepQuantiser tsq;
    tsq_init(&tsq, &out.profile, FP_FROM_FLOAT(0.001f)); // 1 ms steps

    fp_t pos, vel, acc;
    while (tsq_step(&tsq, &pos, &vel, &acc)) {
        // vel maps directly to motor duty cycle or speed command:
        int32_t pwm_duty = FP_TO_INT(fp_mul(vel, FP_FROM_FLOAT(0.01f)));
        set_motor_pwm(pwm_duty);
    }
}
```

---

## Quick-Start: Stepper Motor (Distance-Step Quantiser)

```c
#include "ruckig_fp/ruckig_fp.h"
#include "ruckig_fp/quantise.h"

RuckigFpInput inp = { /* same as above */ };
RuckigFpOutput out;

if (ruckig_fp_calculate(&inp, &out)) {
    DistStepQuantiser dsq;
    dsq_init(&dsq, &out.profile, FP_FROM_INT(1)); // 1 unit = 1 step

    fp_t step_time;
    fp_t prev_time = FP_ZERO;
    while (dsq_next_step_time(&dsq, &step_time)) {
        // Compute the period between this step and the previous
        fp_t period_s = step_time - prev_time;
        // Convert to hardware timer counts (e.g. 125 MHz timer)
        uint32_t counts = (uint32_t)FP_TO_INT(
            fp_mul(period_s, FP_FROM_INT(125000000)));
        load_step_timer(counts);
        prev_time = step_time;
    }
}
```

---

## Build Instructions

### Host (Linux / macOS) for Testing

```bash
mkdir build && cd build
cmake -DBUILD_EXAMPLES=ON ..
make
./ruckig_fp_example
```

### Raspberry Pi Pico (with Pico SDK)

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_gcc.cmake \
      -DBUILD_EXAMPLES=ON ..
make
# Flash ruckig_fp_example.uf2 to your Pico
```

### As a Sub-library in Your Project

In your project's `CMakeLists.txt`:

```cmake
add_subdirectory(ruckig_fp)
target_link_libraries(my_target PRIVATE ruckig_fp)
```

Then include:
```c
#include "ruckig_fp/ruckig_fp.h"
#include "ruckig_fp/quantise.h"
```

---

## API Reference

### `ruckig_fp_calculate(inp, out)` → `bool`

Plans the trajectory.  Returns `true` on success.

### `profile_at_time(prof, t, &pos, &vel, &acc)`

Samples position/velocity/acceleration at time `t` along the profile.

### `profile_total_duration(prof)` → `fp_t`

Returns the total duration of the planned move.

### `tsq_init(q, prof, dt)` / `tsq_step(q, &pos, &vel, &acc)` → `bool`

Time-step quantiser.  Returns `false` when trajectory is complete.

### `dsq_init(q, prof, step_size)` / `dsq_next_step_time(q, &step_time)` → `bool`

Distance-step quantiser.  Returns `false` when all steps are complete.

---

## Known Limitations

1. **Zero initial/final acceleration only.** The closed-form solver assumes
   `a0 = af = 0`.  Non-zero boundary accelerations are not currently
   supported.
2. **Symmetric limits only.** The bisection fallback uses `a_min = −a_max`
   and `v_min = −v_max`.  Asymmetric limits require a more complex solver.
3. **Single axis.** Multi-axis synchronisation is not implemented.
4. **Q16.16 dynamic range.** Positions must fit in ±32 767 units.  Use
   `FP_FRAC_BITS=8` for coarser-resolution high-range applications.
5. **No online re-planning.** The library plans offline; mid-trajectory
   target changes require a new `ruckig_fp_calculate()` call.

---

## License

MIT — see repository root for full text.