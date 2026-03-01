/**
 * @file ruckig_fp.h
 * @brief Public API for the ruckig_fp single-axis jerk-limited trajectory planner.
 *
 * ## Overview
 *
 * `ruckig_fp` plans a third-order (jerk-limited) point-to-point motion profile
 * for a single axis.  Given the current kinematic state, a target state, and
 * physical motion limits, it computes the optimal 7-segment S-curve trajectory
 * and stores it in an `SCurveProfile` structure that can be sampled at any time.
 *
 * ## Coordinate convention
 *
 * All physical quantities use the same unit system as the caller.  For example
 * if positions are in micrometres, velocities are in µm/s, accelerations in
 * µm/s², and jerks in µm/s³.  The library is unit-agnostic; the caller defines
 * the scale via the fixed-point representation (see `fixed_point.h`).
 *
 * ## Fixed-point usage
 *
 * All fields in `RuckigFpInput` and `RuckigFpOutput` are `fp_t` values.  Use
 * the `FP_FROM_INT` and `FP_FROM_FLOAT` macros to construct them:
 *
 * ```c
 * RuckigFpInput inp;
 * inp.p0    = FP_FROM_INT(0);
 * inp.pf    = FP_FROM_INT(1000);
 * inp.v_max = FP_FROM_INT(200);
 * inp.j_max = FP_FROM_INT(2000);
 * // ...
 * ```
 */

#ifndef RUCKIG_FP_H
#define RUCKIG_FP_H

#include <stdbool.h>
#include "ruckig_fp/fixed_point.h"
#include "ruckig_fp/profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Input structure
 * --------------------------------------------------------------------- */

/**
 * @brief Input parameters for the trajectory planner.
 *
 * All values are fixed-point (fp_t).  The caller is responsible for ensuring
 * the limits are physically consistent (e.g. j_max > 0, a_max > 0).
 */
typedef struct {
    /** Current (initial) position. */
    fp_t p0;

    /** Current (initial) velocity.  Must satisfy v_min <= v0 <= v_max. */
    fp_t v0;

    /** Current (initial) acceleration.  Must satisfy a_min <= a0 <= a_max. */
    fp_t a0;

    /** Target (final) position. */
    fp_t pf;

    /** Target (final) velocity.  Must satisfy v_min <= vf <= v_max.
     *  Set to FP_ZERO for a stop-at-target move. */
    fp_t vf;

    /** Target (final) acceleration.  Set to FP_ZERO for a typical move. */
    fp_t af;

    /** Maximum velocity (positive).  Must be > 0. */
    fp_t v_max;

    /** Minimum velocity (may be negative for bidirectional axes).
     *  For a unidirectional axis set to FP_ZERO or a small positive value. */
    fp_t v_min;

    /** Maximum acceleration (positive).  Must be > 0. */
    fp_t a_max;

    /** Minimum acceleration (negative for symmetric limits, e.g. -a_max).
     *  Must be < 0. */
    fp_t a_min;

    /** Maximum jerk (positive).  Must be > 0. */
    fp_t j_max;

    /**
     * Velocity mode flag.
     *
     * When false (default), the trajectory decelerates to `vf` at `pf` and
     * stops (or holds `vf` at the boundary — no further motion).
     *
     * When true, the trajectory arrives at `pf` with velocity `vf` and then
     * continues at constant velocity `vf` indefinitely (zero acceleration,
     * position increasing linearly at `vf`).  The quantisers will never
     * report "finished" in this mode; the caller must explicitly stop or
     * replan the trajectory.
     *
     * Typical use: arrive at a sync/handoff position at a specified speed,
     * then follow a conveyor or continue with the next move.
     */
    bool vel_mode;
} RuckigFpInput;

/* -------------------------------------------------------------------------
 * Output structure
 * --------------------------------------------------------------------- */

/**
 * @brief Output of the trajectory planner.
 *
 * If `valid` is true, `profile` contains a fully integrated S-curve that
 * moves the axis from the initial to the target state within the specified
 * limits.  The profile can be sampled with `profile_at_time()` or stepped
 * through with the quantiser API in `quantise.h`.
 */
typedef struct {
    /** The computed S-curve profile.  Valid only when `valid == true`. */
    SCurveProfile profile;

    /** True if a physically valid trajectory was found, false otherwise. */
    bool valid;
} RuckigFpOutput;

/* -------------------------------------------------------------------------
 * Main API
 * --------------------------------------------------------------------- */

/**
 * @brief Plan a jerk-limited trajectory from the current state to the target.
 *
 * The function attempts to find an S-curve that:
 *   - Starts at (p0, v0, a0) — a0 may be non-zero (e.g. for online replanning)
 *   - Ends at (pf, vf, af)
 *   - Respects [v_min, v_max], [a_min, a_max], and j_max
 *   - Minimises total travel time (time-optimal within the given limits)
 *
 * ### Handling non-zero a0
 *
 * When `a0 != 0`, a single "zeroing" segment is prepended before the standard
 * 7-segment S-curve.  This segment applies jerk `-sign(a0) * j_max` for
 * duration `|a0| / j_max` to bring the acceleration to zero before the main
 * trajectory begins.  The total profile then has up to 8 segments.
 *
 * **Limitation:** if zeroing `a0` would cause the instantaneous velocity to
 * exceed `[v_min, v_max]`, the function returns false.  In practice this only
 * occurs when `v0` is already near `v_max` AND `a0` is near `a_max`; for
 * typical online-replanning scenarios (small `a0`, frequent control-loop
 * updates) this is not an issue.
 *
 * ### Velocity mode
 *
 * When `inp->vel_mode` is true the trajectory arrives at `pf` at velocity `vf`
 * and then continues at that velocity indefinitely (constant-velocity cruise).
 * See `RuckigFpInput::vel_mode` and `SCurveProfile::vel_mode` for details.
 *
 * @param inp  Pointer to the input parameters (must not be NULL).
 * @param out  Pointer to the output structure to fill (must not be NULL).
 * @return     true if a valid trajectory was found and stored in out->profile,
 *             false if the inputs are inconsistent or no solution exists.
 */
bool ruckig_fp_calculate(const RuckigFpInput *inp, RuckigFpOutput *out);

/**
 * @brief Online replan: sample the current trajectory and plan a new one.
 *
 * Convenience wrapper for online trajectory updates.  Evaluates the
 * currently-running `current_out->profile` at `elapsed_time` to obtain the
 * instantaneous kinematic state (position, velocity, acceleration), then
 * uses that state as the initial conditions for a fresh `ruckig_fp_calculate()`
 * call toward the new target described in `new_inp`.
 *
 * The limits (`v_max`, `a_max`, `j_max`, etc.) and the new target (`pf`,
 * `vf`, `af`, `vel_mode`) are taken from `new_inp`; only `p0`, `v0`, `a0`
 * are overridden with the sampled state.
 *
 * Typical usage in a 1 kHz control loop:
 * ```c
 * fp_t t_now = FP_FROM_FLOAT(0.001f * loop_count);
 * RuckigFpInput new_inp = { .pf = new_target, .vf = 0, .af = 0,
 *                            .v_max = ..., .a_max = ..., .j_max = ... };
 * RuckigFpOutput new_out;
 * if (ruckig_fp_replan(t_now, &current_out, &new_inp, &new_out)) {
 *     current_out = new_out;   // switch to the new trajectory
 *     loop_count  = 0;         // reset time reference
 * }
 * ```
 *
 * @param elapsed_time  Time elapsed since the start of `current_out->profile`
 *                      (fixed-point seconds).
 * @param current_out   The currently executing trajectory output.  Must be
 *                      valid (`current_out->valid == true`).
 * @param new_inp       New target and limits.  `p0`, `v0`, `a0` are ignored
 *                      and replaced by the sampled state.
 * @param new_out       Output: the replanned trajectory.
 * @return              true if a valid new trajectory was computed.
 */
bool ruckig_fp_replan(fp_t elapsed_time,
                      const RuckigFpOutput *current_out,
                      const RuckigFpInput  *new_inp,
                      RuckigFpOutput       *new_out);

#ifdef __cplusplus
}
#endif

#endif /* RUCKIG_FP_H */
