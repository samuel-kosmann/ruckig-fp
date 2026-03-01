/**
 * @file quantise.h
 * @brief Time-step and distance-step quantisation iterators for ruckig_fp.
 *
 * After planning a trajectory with `ruckig_fp_calculate()`, the resulting
 * `SCurveProfile` must be "played back" to drive physical hardware.  This
 * header provides two iterator-style structs for the two most common output
 * modes on embedded motor-control systems:
 *
 * ## Time-Step Quantiser (`TimeStepQuantiser`)
 *
 * Samples the profile at regular time intervals `dt`.  Suitable for:
 *   - **DC motors** (PWM duty-cycle or analogue speed reference)
 *   - **Servo drives** (velocity or position command updated at the servo
 *     loop period)
 *   - Any system where the control loop runs at a fixed rate
 *
 * The output `vel` value maps directly to the motor speed command.  Choose
 * `dt` equal to the control-loop period (e.g. FP_FROM_FLOAT(0.001f) for 1 ms).
 *
 * **Error accumulation note:** Fixed-point time accumulation introduces up
 * to 1 LSB of error per step.  Over N steps this grows to at most N LSBs,
 * which for Q16.16 at 1 ms steps over a 10-second move is ~655,000 steps ×
 * 1/65536 s = ~10 ms of timing error.  For most control applications this
 * is negligible; if higher accuracy is needed, use a higher-precision timer
 * and cross-check the trajectory endpoint with `profile_total_duration()`.
 *
 * ## Distance-Step Quantiser (`DistStepQuantiser`)
 *
 * Finds the exact time at which each integer step position is reached.
 * Suitable for:
 *   - **Stepper motors** (the returned time is when the next step pulse
 *     should fire; the interval between successive calls gives the step
 *     period, which is loaded directly into a hardware timer)
 *   - **Encoder position control** (step events correspond to encoder ticks)
 *
 * Uses a **bisection search** within each segment to find the exact crossing
 * time without any floating-point arithmetic.  The bisection converges in
 * at most 32 iterations (one per bit of fp_t precision), making it safe
 * and deterministic on the RP2040.
 *
 * **Precision / granularity trade-off:** `step_size` sets the distance per
 * step.  FP_FROM_INT(1) gives 1 unit per step.  If your physical unit is
 * microsteps, this directly gives microstep timing.  Finer granularity
 * (smaller step_size) increases the number of calls and the total
 * computation time, but does not degrade numerical accuracy.
 */

#ifndef RUCKIG_FP_QUANTISE_H
#define RUCKIG_FP_QUANTISE_H

#include <stdbool.h>
#include "ruckig_fp/fixed_point.h"
#include "ruckig_fp/profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Time-Step Quantiser
 * ======================================================================= */

/**
 * @brief State for the time-step quantiser iterator.
 *
 * Maintains a time cursor that advances by `dt` on each call to `tsq_step()`.
 * No heap allocation; the struct can be stack-allocated.
 */
typedef struct {
    /** Pointer to the profile being played back (not owned). */
    const SCurveProfile *profile;

    /** Current time along the trajectory (fixed-point seconds). */
    fp_t current_time;

    /**
     * Fixed time step between output samples (fixed-point seconds).
     *
     * Should equal the control-loop period.  Smaller values give smoother
     * output but increase the number of iterations (and ISR overhead for
     * timer-driven loops).
     */
    fp_t dt;

    /** True once the trajectory has been fully played back. */
    bool finished;
} TimeStepQuantiser;

/**
 * @brief Initialise a time-step quantiser.
 *
 * Must be called before the first call to `tsq_step()`.
 *
 * @param q     Quantiser to initialise (must not be NULL).
 * @param prof  Profile to play back (must be valid and already integrated).
 * @param dt    Time step in fixed-point seconds (e.g. FP_FROM_FLOAT(0.001f)
 *              for a 1 ms control loop).  Must be > 0.
 */
void tsq_init(TimeStepQuantiser *q, const SCurveProfile *prof, fp_t dt);

/**
 * @brief Advance the quantiser by one time step and return the kinematic state.
 *
 * Evaluates the profile at the current time, then advances `current_time`
 * by `dt` for the next call.
 *
 * **DC motor usage:** Map the returned `vel` to a PWM duty cycle or analogue
 * speed reference.  `pos` can be used as a feedforward position reference.
 * `acc` is useful for feedforward torque/current control.
 *
 * @param q    Quantiser state (must have been initialised with tsq_init()).
 * @param pos  Output: position at the current time step (may be NULL).
 * @param vel  Output: velocity at the current time step (may be NULL).
 * @param acc  Output: acceleration at the current time step (may be NULL).
 * @return     true if the trajectory is still in progress (more steps remain),
 *             false when the trajectory is complete (all subsequent calls also
 *             return false and output the final state).
 */
bool tsq_step(TimeStepQuantiser *q, fp_t *pos, fp_t *vel, fp_t *acc);

/* =========================================================================
 * Distance-Step Quantiser
 * ======================================================================= */

/**
 * @brief State for the distance-step quantiser iterator.
 *
 * Maintains a position cursor and a time cursor.  On each call to
 * `dsq_next_step_time()` a bisection search finds the exact time when the
 * next integer step boundary is crossed.
 */
typedef struct {
    /** Pointer to the profile being played back (not owned). */
    const SCurveProfile *profile;

    /**
     * The next target step position (fixed-point, integer multiple of
     * step_size).  Starts at p[0] + step_size after initialisation.
     */
    fp_t next_step_pos;

    /**
     * Distance per step in fixed-point units.
     *
     * - FP_FROM_INT(1) gives one step per unit of position.
     * - FP_FROM_FLOAT(0.001f) gives one step per millimetre when positions
     *   are in micrometres.
     *
     * **Mapping to physical microsteps:** if your stepper has 200 full steps/rev
     * and you are commanding 16× microstepping, with position in units of
     * full-step then step_size = FP_FROM_FLOAT(1.0f/16).
     */
    fp_t step_size;

    /**
     * Lower bound for the bisection search (time cursor).
     * Advances to the time of the previous step after each call.
     */
    fp_t current_time;

    /** True once the final position has been reached. */
    bool finished;
} DistStepQuantiser;

/**
 * @brief Initialise a distance-step quantiser.
 *
 * @param q          Quantiser to initialise (must not be NULL).
 * @param prof       Profile to play back (must be valid and already integrated).
 * @param step_size  Distance between consecutive step pulses (fixed-point).
 *                   Must be > 0.
 */
void dsq_init(DistStepQuantiser *q, const SCurveProfile *prof, fp_t step_size);

/**
 * @brief Find the time of the next step position crossing.
 *
 * Uses a 32-iteration bisection to locate the exact time within [current_time,
 * total_duration] at which position equals `next_step_pos`.
 *
 * **Bisection algorithm:**
 *   1. Set lo = current_time, hi = total_duration.
 *   2. If p(hi) < next_step_pos, the step is never reached: return false.
 *   3. Repeat 32 times:
 *      a. mid = (lo + hi) / 2
 *      b. If p(mid) < next_step_pos: lo = mid, else hi = mid.
 *   4. step_time = hi (the first time p >= next_step_pos).
 *
 * After the call `current_time` is advanced to `step_time` and
 * `next_step_pos` is advanced by `step_size`.
 *
 * **Why 32 iterations?** Each iteration halves the search interval.  Starting
 * from [0, T] (T up to ~32767 seconds in Q16.16), after 32 iterations the
 * interval width is T / 2^32 ≈ T × 2.3e-10 seconds, well below the 1-LSB
 * resolution of Q16.16 (≈ 15 µs).  Convergence is guaranteed and deterministic.
 *
 * **Stepper motor usage:** The interval between successive `step_time` values
 * is the period between step pulses.  Load this interval (converted to timer
 * counts) into a hardware timer to generate the step signal:
 * ```c
 * fp_t t_prev = FP_ZERO, t_curr;
 * while (dsq_next_step_time(&dsq, &t_curr)) {
 *     fp_t period = t_curr - t_prev;
 *     uint32_t timer_counts = FP_TO_INT(fp_mul(period, FP_FROM_INT(TIMER_FREQ)));
 *     load_step_timer(timer_counts);
 *     t_prev = t_curr;
 * }
 * ```
 *
 * @param q          Quantiser state.
 * @param step_time  Output: time at which the next step position is reached.
 * @return           true if the step was found, false if the trajectory is
 *                   complete (no more steps remain).
 */
bool dsq_next_step_time(DistStepQuantiser *q, fp_t *step_time);

#ifdef __cplusplus
}
#endif

#endif /* RUCKIG_FP_QUANTISE_H */
