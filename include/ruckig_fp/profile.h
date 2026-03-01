/**
 * @file profile.h
 * @brief S-curve (7-segment jerk-limited) motion profile data structures.
 *
 * Defines the `SCurveProfile` struct that holds the complete kinematic
 * description of a jerk-limited point-to-point move, and declares the
 * functions that operate on it.
 *
 * ## 7-Segment UDDU Pattern
 *
 * A full S-curve move is decomposed into 7 segments with alternating jerk
 * signs.  The "UDDU" naming refers to the sign pattern of jerk as seen from
 * the perspective of positive displacement:
 *
 * ```
 * Segment: 1      2      3      4      5      6      7
 * Phase:   Accel  Accel  Accel  Coast  Decel  Decel  Decel
 * Jerk:    +J     0      -J     0      -J     0      +J
 * ```
 *
 * - Segments 1-3 bring velocity from v0 up to v_peak.
 * - Segment 4 is the optional constant-velocity coast phase.
 * - Segments 5-7 bring velocity from v_peak back down to vf.
 *
 * Some segments may have zero duration, yielding shorter profiles:
 * - No coast phase: t[3] = 0
 * - No constant-acceleration phase (triangular accel): t[1] = 0, t[5] = 0
 *
 * ## Coordinate Convention
 *
 * All positions, velocities, accelerations, and jerks use the same physical
 * units as the caller (the planner works in "counts", "mm", "steps", etc.).
 * Time is in seconds (or ticks — whatever the caller defines).
 */

#ifndef RUCKIG_FP_PROFILE_H
#define RUCKIG_FP_PROFILE_H

#include "ruckig_fp/fixed_point.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Standard number of segments in the 7-segment S-curve (kept for compatibility). */
#define SCURVE_SEGMENTS     7

/**
 * @brief Maximum number of segments that a profile may contain.
 *
 * Up to 9 segments are used when a non-zero initial acceleration (`a0`) is
 * present (one prepended "zeroing" segment before the standard 7-segment
 * S-curve) and/or when a non-zero final acceleration (`af`) is requested
 * (one appended "ramping" segment).
 *
 *   n_segs breakdown:
 *     7 = standard S-curve (a0 == 0, af == 0)
 *     8 = one pre-segment (a0 != 0) OR one post-segment (af != 0)
 *     9 = both pre- and post-segments
 */
#define SCURVE_MAX_SEGMENTS 9

/**
 * @brief Complete description of a jerk-limited S-curve profile.
 *
 * The profile is fully specified by the segment durations `t[]` and jerk
 * values `j[]`.  The boundary conditions `a[]`, `v[]`, `p[]` are derived
 * by calling `profile_integrate()`.
 *
 * `n_segs` indicates how many entries of `t[]` and `j[]` are active (1..9).
 * Only indices 0 .. n_segs-1 are valid.
 *
 * Array sizing convention:
 *   - `t[i]`, `j[i]`  —  indexed 0 .. n_segs-1
 *   - `a[i]`, `v[i]`, `p[i]`  —  indexed 0 .. n_segs  (n_segs+1 boundary points)
 *
 * ## Velocity mode
 *
 * When `vel_mode` is true the trajectory does not stop at the final position.
 * After the last segment the axis continues at constant velocity `v[n_segs]`
 * indefinitely.  `profile_at_time()` will extrapolate correctly for query
 * times beyond the total duration, and the quantisers will never report
 * "finished".  The caller must stop or replan explicitly.
 */
typedef struct {
    /**
     * Number of active segments (1 .. SCURVE_MAX_SEGMENTS).
     * Set by the planner; the profile API reads this to know how many
     * segments to iterate.
     */
    int n_segs;

    /** Duration of each segment (fixed-point seconds). */
    fp_t t[SCURVE_MAX_SEGMENTS];

    /** Jerk applied during each segment (fixed-point units/s^3). */
    fp_t j[SCURVE_MAX_SEGMENTS];

    /** Acceleration at each segment boundary (units/s^2). a[0] = initial. */
    fp_t a[SCURVE_MAX_SEGMENTS + 1];

    /** Velocity at each segment boundary (units/s). v[0] = initial. */
    fp_t v[SCURVE_MAX_SEGMENTS + 1];

    /** Position at each segment boundary (units). p[0] = initial. */
    fp_t p[SCURVE_MAX_SEGMENTS + 1];

    /** Target final position (set by planner, used for validation). */
    fp_t pf;

    /** Target final velocity (set by planner). */
    fp_t vf;

    /** Target final acceleration (set by planner). */
    fp_t af;

    /**
     * Velocity mode flag.  When true, the trajectory continues at constant
     * velocity `v[n_segs]` beyond the final position.  Set from
     * `RuckigFpInput::vel_mode` by the planner.
     */
    bool vel_mode;
} SCurveProfile;

/* -------------------------------------------------------------------------
 * Profile API
 * --------------------------------------------------------------------- */

/**
 * @brief Integrate segment durations and jerks to fill boundary conditions.
 *
 * Given the `t[]` and `j[]` arrays already filled in for indices 0 .. n_segs-1,
 * and `a[0]`, `v[0]`, `p[0]` set as initial conditions, this function computes
 * `a[1..n_segs]`, `v[1..n_segs]`, and `p[1..n_segs]` using the kinematic
 * update equations:
 *
 * ```
 *   a[i+1] = a[i] + t[i] * j[i]
 *   v[i+1] = v[i] + t[i]*a[i] + t[i]^2*j[i] / 2
 *   p[i+1] = p[i] + t[i]*v[i] + t[i]^2*a[i]/2 + t[i]^3*j[i]/6
 * ```
 *
 * All arithmetic uses the fixed-point helpers (fp_mul, fp_div) to stay
 * within 32-bit intermediates where possible.
 *
 * @param prof  Pointer to the profile.  `n_segs`, `t[]`, `j[]`, `a[0]`, `v[0]`,
 *              `p[0]` must be initialised before calling.
 */
void profile_integrate(SCurveProfile *prof);

/**
 * @brief Query the kinematic state at an arbitrary time along the profile.
 *
 * Uses a linear search over the segments (at most SCURVE_MAX_SEGMENTS = 9)
 * to find which segment contains `t_query`, then evaluates the kinematic
 * polynomial within that segment.
 *
 * When `prof->vel_mode` is true and `t_query` exceeds the total profile
 * duration, the function extrapolates at constant velocity `v[n_segs]`
 * (zero acceleration) — modelling indefinite cruise at the final velocity.
 *
 * @param prof     Pointer to the (already integrated) profile.
 * @param t_query  Time from the start of the profile (fixed-point seconds).
 *                 Clamped to [0, total_duration] when not in velocity mode.
 * @param pos      Output: position at t_query (may be NULL).
 * @param vel      Output: velocity at t_query (may be NULL).
 * @param acc      Output: acceleration at t_query (may be NULL).
 */
void profile_at_time(const SCurveProfile *prof, fp_t t_query,
                     fp_t *pos, fp_t *vel, fp_t *acc);

/**
 * @brief Sum the durations of all segments.
 *
 * @param prof  Pointer to the profile.
 * @return      Total motion duration in fixed-point seconds.
 */
fp_t profile_total_duration(const SCurveProfile *prof);

#ifdef __cplusplus
}
#endif

#endif /* RUCKIG_FP_PROFILE_H */
