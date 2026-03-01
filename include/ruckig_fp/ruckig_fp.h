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
 * The function attempts to find a 7-segment UDDU S-curve that:
 *   - Starts at (p0, v0, a0)
 *   - Ends at (pf, vf, af)
 *   - Respects [v_min, v_max], [a_min, a_max], and j_max
 *   - Minimises total travel time (time-optimal within the given limits)
 *
 * The algorithm:
 *   1. Computes the required displacement and chooses the motion direction.
 *   2. Tries the full 7-segment profile (velocity limited + accel limited).
 *   3. Falls back to 5 segments (no coast phase) if v_max is not reached.
 *   4. Falls back to 3 segments (no constant-accel phase) if needed.
 *   5. Handles the trivial zero-displacement case.
 *
 * @param inp  Pointer to the input parameters (must not be NULL).
 * @param out  Pointer to the output structure to fill (must not be NULL).
 * @return     true if a valid trajectory was found and stored in out->profile,
 *             false if the inputs are inconsistent or no solution exists.
 */
bool ruckig_fp_calculate(const RuckigFpInput *inp, RuckigFpOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* RUCKIG_FP_H */
