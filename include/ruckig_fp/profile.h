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

/** Number of segments in the 7-segment S-curve */
#define SCURVE_SEGMENTS 7

/**
 * @brief Complete description of a 7-segment jerk-limited S-curve profile.
 *
 * The profile is fully specified by the segment durations `t[]` and jerk
 * values `j[]`.  The boundary conditions `a[]`, `v[]`, `p[]` are derived
 * by calling `profile_integrate()`.
 *
 * Array sizing convention:
 *   - `t[i]`, `j[i]`  —  indexed 0..6  (one entry per segment)
 *   - `a[i]`, `v[i]`, `p[i]`  —  indexed 0..7  (8 boundary points:
 *     start of segment 0 … start of segment 7 = end of segment 6)
 */
typedef struct {
    /** Duration of each of the 7 segments (fixed-point seconds). */
    fp_t t[SCURVE_SEGMENTS];

    /** Jerk applied during each segment (fixed-point units/s^3). */
    fp_t j[SCURVE_SEGMENTS];

    /** Acceleration at each segment boundary (units/s^2). a[0] = initial. */
    fp_t a[SCURVE_SEGMENTS + 1];

    /** Velocity at each segment boundary (units/s). v[0] = initial. */
    fp_t v[SCURVE_SEGMENTS + 1];

    /** Position at each segment boundary (units). p[0] = initial. */
    fp_t p[SCURVE_SEGMENTS + 1];

    /** Target final position (set by planner, used for validation). */
    fp_t pf;

    /** Target final velocity (set by planner). */
    fp_t vf;

    /** Target final acceleration (set by planner). */
    fp_t af;
} SCurveProfile;

/* -------------------------------------------------------------------------
 * Profile API
 * --------------------------------------------------------------------- */

/**
 * @brief Integrate segment durations and jerks to fill boundary conditions.
 *
 * Given the `t[]` and `j[]` arrays already filled in, and `a[0]`, `v[0]`,
 * `p[0]` set as initial conditions, this function computes `a[1..7]`,
 * `v[1..7]`, and `p[1..7]` using the kinematic update equations:
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
 * @param prof  Pointer to the profile.  `t[]`, `j[]`, `a[0]`, `v[0]`,
 *              `p[0]` must be initialised before calling.
 */
void profile_integrate(SCurveProfile *prof);

/**
 * @brief Query the kinematic state at an arbitrary time along the profile.
 *
 * Uses a linear search over the 7 segments (the profile is short enough
 * that a binary search would add code complexity for negligible gain).
 * Evaluates the jerk-kinematics polynomial within the located segment.
 *
 * @param prof     Pointer to the (already integrated) profile.
 * @param t_query  Time from the start of the profile (fixed-point seconds).
 *                 Clamped to [0, total_duration] internally.
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
