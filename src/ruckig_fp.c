/**
 * @file ruckig_fp.c
 * @brief Single-axis jerk-limited trajectory planner (fixed-point).
 *
 * ## Algorithm Overview
 *
 * We compute a "time-optimal" 7-segment S-curve trajectory subject to
 * velocity, acceleration, and jerk limits.  The approach is the standard
 * analytical (closed-form) solution used in industrial motion controllers.
 *
 * ### Profile Types
 *
 * Starting from the most-capable (longest) profile and falling back:
 *
 * 1. **Type 1 — Full 7-segment (velocity-limited + accel-limited)**
 *    Both v_max and a_max are reached.  All 7 segment durations are > 0.
 *    Segment structure: [accel-jerk][const-accel][decel-jerk][coast]
 *                       [decel-jerk][const-decel][accel-jerk]
 *
 * 2. **Type 2 — 5-segment (velocity-limited, no coast)**
 *    v_max is reached but the profile is symmetric enough that no coast
 *    phase is needed (t4 = 0).
 *
 * 3. **Type 3 — 5-segment (accel-limited, no peak velocity)**
 *    a_max is reached on both ramps but v_max is NOT reached.
 *    Velocity rises to an intermediate peak v_peak < v_max.
 *
 * 4. **Type 4 — 3-segment (pure jerk)**
 *    Neither a_max nor v_max is reached.  Just three jerk segments.
 *
 * 5. **Trivial — zero displacement**
 *    p0 == pf and v0 == vf == 0.
 *
 * ### Simplifying Assumptions
 *
 * - Initial and final accelerations (a0, af) are both zero (simplest case
 *   handled by the solver; non-zero a0/af require more complex algebra and
 *   are currently out of scope).
 * - The motion direction is determined by (pf - p0): positive means forward
 *   (UDDU jerk pattern), negative means backward (all jerks/accelerations
 *   are negated).
 * - Only symmetric limits are supported in the fallback paths:
 *   a_min = -a_max, v_min = -v_max.
 *
 * ### Mathematics
 *
 * For a positive-direction UDDU profile the seven segments have jerk:
 *   j = [+J, 0, -J, 0, -J, 0, +J]
 *
 * where J = j_max, A = a_max.  Denoting segment durations t1..t7:
 *
 *   Acceleration ramp (segments 1-3):
 *     t1 = t3 = A / J      (time to ramp up/down acceleration)
 *     t2 = (V_peak - v0 - A*t1) / A   [const-accel coast; may be 0]
 *     v_after_accel = v0 + A*t1 + A*t2 + (-J)*(t3^2)/2 - J*t3*... etc.
 *
 * More precisely, using the closed-form:
 *
 *   v_reached_by_ramp = v0 + A^2/J   (when t2=0, triangular ramp)
 *   If V_peak <= v0 + A^2/J:
 *     t1 = t3 = sqrt((V_peak - v0) / J)   (no const-accel phase)
 *     t2 = 0
 *   Else:
 *     t1 = t3 = A / J
 *     t2 = (V_peak - v0) / A - A / J
 *
 *   Similarly for the deceleration ramp (segments 5-7):
 *   If V_peak <= vf + A^2/J:
 *     t5 = t7 = sqrt((V_peak - vf) / J)
 *     t6 = 0
 *   Else:
 *     t5 = t7 = A / J
 *     t6 = (V_peak - vf) / A - A / J
 *
 *   Distance covered by the acceleration ramp:
 *     d_accel = v0*(t1+t2+t3) + ...  (evaluated by integrating the kinematics)
 *   Similarly for deceleration ramp d_decel.
 *
 *   Coast phase:
 *     d_coast = pf - p0 - d_accel - d_decel
 *     t4 = d_coast / V_peak
 *
 * If t4 < 0, V_peak must be reduced (binary search or closed-form solve).
 * We use a simple bisection on V_peak in [max(v0,vf), v_max].
 *
 * No <math.h> or floating-point is used in this file.
 */

#include <stddef.h>
#include "ruckig_fp/ruckig_fp.h"

/* Small epsilon for fixed-point zero comparisons */
#define FP_EPS  ((fp_t)4)   /* ~0.00006 in Q16.16 */

/* -------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

/**
 * Compute the distance covered by an acceleration or deceleration ramp
 * from velocity v_start to velocity v_end using jerk limit J and accel limit A.
 *
 * The ramp is a symmetric 3-phase manoeuvre (jerk-ramp / const-accel / jerk-ramp)
 * that begins and ends with zero acceleration.  By symmetry, the average velocity
 * over the entire ramp is exactly (v_start + v_end) / 2, so:
 *
 *   distance = (v_start + v_end) / 2  *  T_ramp
 *
 * where T_ramp is:
 *   - Triangular (|dv| <= A^2/J):  T = 2 * sqrt(|dv| / J)
 *   - Trapezoidal (|dv| > A^2/J):  T = A/J + |dv|/A
 *
 * This formula is exact for both acceleration ramps (v_end > v_start) and
 * deceleration ramps (v_end < v_start), and requires no per-segment integrals.
 *
 * Derivation sketch (trapezoidal, accel ramp from v0 to v_peak):
 *   Phase 1: j=+J, t1=A/J.  Distance = v0*t1 + J*t1^3/6.
 *   Phase 2: j=0,  t2=dv/A - A/J.  Distance = v1*t2 + A*t2^2/2.
 *   Phase 3: j=-J, t3=A/J.  Distance = v2*t3 + A*t3^2/2 - J*t3^3/6.
 *   Sum = (v0+v_peak)/2 * (2*A/J + dv/A - A/J) = (v0+v_peak)/2 * (A/J + dv/A). QED
 *
 * @param v_start  Starting velocity (non-negative, in the positive-direction domain).
 * @param v_end    Ending velocity (non-negative).
 * @param A        Acceleration limit (positive).
 * @param J        Jerk limit (positive).
 * @param out_dur  Output: total ramp duration T_ramp (may be NULL).
 * @return         Distance covered by the ramp (always non-negative).
 */
static fp_t ramp_distance(fp_t v_start, fp_t v_end, fp_t A, fp_t J,
                           fp_t *out_dur)
{
    fp_t delta_v = fp_abs(v_end - v_start);
    fp_t A2_over_J = fp_div(fp_mul(A, A), J); /* A^2/J */
    fp_t T;

    if (delta_v <= A2_over_J + FP_EPS) {
        /* Triangular ramp: T = 2*sqrt(dv/J) */
        fp_t ta = fp_sqrt(fp_div(fp_max(delta_v, FP_ZERO), J));
        T = fp_mul(FP_TWO, ta);
    } else {
        /* Trapezoidal ramp: T = A/J + dv/A */
        fp_t ta = fp_div(A, J);
        fp_t tb = fp_div(delta_v, A) - ta;
        T = fp_mul(FP_TWO, ta) + tb;
    }

    if (out_dur) *out_dur = T;

    /* distance = average_velocity * duration = (v_start + v_end) / 2 * T */
    return fp_div(fp_mul(v_start + v_end, T), FP_TWO);
}

/**
 * Compute the 7 segment durations for a given peak velocity v_peak,
 * writing them into prof->t[] and prof->j[].
 * Returns true if all durations are non-negative (physically valid).
 *
 * @param prof    Profile to fill in.
 * @param v0      Initial velocity.
 * @param vf      Final velocity.
 * @param dp      Displacement (pf - p0), must be positive (caller negates).
 * @param v_peak  Desired peak velocity (>= max(v0,vf), <= v_max).
 * @param A       Acceleration limit (positive).
 * @param J       Jerk limit (positive).
 * @param dir     Direction: +1 or -1.
 * @return        true if t4 >= 0 (coast phase non-negative).
 */
static bool compute_segments(SCurveProfile *prof,
                              fp_t v0, fp_t vf, fp_t dp,
                              fp_t v_peak, fp_t A, fp_t J, int dir)
{
    fp_t dur_accel = FP_ZERO;
    fp_t dur_decel = FP_ZERO;

    fp_t d_accel = ramp_distance(v0, v_peak, A, J, &dur_accel);
    fp_t d_decel = ramp_distance(v_peak, vf, A, J, &dur_decel);

    fp_t d_coast = dp - d_accel - d_decel;

    if (d_coast < -FP_EPS) {
        /* v_peak is too high; coast would be negative */
        return false;
    }
    if (d_coast < FP_ZERO) d_coast = FP_ZERO;

    /* Coast time */
    fp_t t4 = (v_peak > FP_EPS) ? fp_div(d_coast, v_peak) : FP_ZERO;

    /* --- Decompose acceleration ramp into segments 1,2,3 --- */
    fp_t dv_accel = v_peak - v0;
    fp_t A2_J = fp_div(fp_mul(A, A), J);
    fp_t t1, t2, t3;

    if (dv_accel <= A2_J + FP_EPS) {
        /* Triangular: t1=t3=sqrt(dv/J), t2=0 */
        t1 = fp_sqrt(fp_div(fp_max(dv_accel, FP_ZERO), J));
        t2 = FP_ZERO;
        t3 = t1;
    } else {
        t1 = fp_div(A, J);
        t2 = fp_div(dv_accel, A) - t1;
        t3 = t1;
    }

    /* --- Decompose deceleration ramp into segments 5,6,7 --- */
    fp_t dv_decel = v_peak - vf;
    fp_t t5, t6, t7;

    if (dv_decel <= A2_J + FP_EPS) {
        t5 = fp_sqrt(fp_div(fp_max(dv_decel, FP_ZERO), J));
        t6 = FP_ZERO;
        t7 = t5;
    } else {
        t5 = fp_div(A, J);
        t6 = fp_div(dv_decel, A) - t5;
        t7 = t5;
    }

    /* Sanity-check all durations non-negative */
    if (t1 < -FP_EPS || t2 < -FP_EPS || t3 < -FP_EPS ||
        t4 < -FP_EPS || t5 < -FP_EPS || t6 < -FP_EPS || t7 < -FP_EPS) {
        return false;
    }
    if (t1 < FP_ZERO) t1 = FP_ZERO;
    if (t2 < FP_ZERO) t2 = FP_ZERO;
    if (t3 < FP_ZERO) t3 = FP_ZERO;
    if (t4 < FP_ZERO) t4 = FP_ZERO;
    if (t5 < FP_ZERO) t5 = FP_ZERO;
    if (t6 < FP_ZERO) t6 = FP_ZERO;
    if (t7 < FP_ZERO) t7 = FP_ZERO;

    /* Write segments (UDDU jerk pattern) */
    /* dir == +1: j = [+J, 0, -J, 0, -J, 0, +J] */
    fp_t Jd = (dir > 0) ? J : -J;
    prof->t[0] = t1; prof->j[0] = +Jd;
    prof->t[1] = t2; prof->j[1] = FP_ZERO;
    prof->t[2] = t3; prof->j[2] = -Jd;
    prof->t[3] = t4; prof->j[3] = FP_ZERO;
    prof->t[4] = t5; prof->j[4] = -Jd;
    prof->t[5] = t6; prof->j[5] = FP_ZERO;
    prof->t[6] = t7; prof->j[6] = +Jd;

    return true;
}

/* -------------------------------------------------------------------------
 * ruckig_fp_calculate
 * --------------------------------------------------------------------- */

bool ruckig_fp_calculate(const RuckigFpInput *inp, RuckigFpOutput *out)
{
    /* Input validation */
    if (!inp || !out) return false;
    if (inp->j_max <= FP_ZERO) return false;
    if (inp->a_max <= FP_ZERO) return false;
    if (inp->v_max <= FP_ZERO) return false;

    out->valid = false;

    fp_t dp = inp->pf - inp->p0;  /* total displacement */

    /* Determine motion direction */
    int dir;
    if (dp > FP_EPS) {
        dir = 1;
    } else if (dp < -FP_EPS) {
        dir = -1;
    } else {
        /* Zero (or near-zero) displacement — trivial profile */
        SCurveProfile *prof = &out->profile;
        for (int i = 0; i < SCURVE_SEGMENTS; i++) {
            prof->t[i] = FP_ZERO;
            prof->j[i] = FP_ZERO;
        }
        prof->a[0] = inp->a0;
        prof->v[0] = inp->v0;
        prof->p[0] = inp->p0;
        prof->pf = inp->pf;
        prof->vf = inp->vf;
        prof->af = inp->af;
        profile_integrate(prof);
        out->valid = true;
        return true;
    }

    /* Work in the positive-displacement domain; negate at the end if dir<0 */
    fp_t dp_abs = fp_abs(dp);
    fp_t v0  = (dir > 0) ? inp->v0 : -inp->v0;
    fp_t vf  = (dir > 0) ? inp->vf : -inp->vf;
    fp_t A   = inp->a_max;                  /* positive acceleration limit */
    fp_t J   = inp->j_max;                  /* positive jerk limit */
    fp_t Vmax = inp->v_max;

    /* Clamp v0 and vf to [0, Vmax] for the simplified solver */
    v0 = fp_max(FP_ZERO, fp_min(v0, Vmax));
    vf = fp_max(FP_ZERO, fp_min(vf, Vmax));

    SCurveProfile *prof = &out->profile;
    prof->a[0] = FP_ZERO; /* simplified: assume zero initial accel */
    prof->v[0] = v0;
    prof->p[0] = inp->p0;
    prof->pf   = inp->pf;
    prof->vf   = inp->vf;
    prof->af   = inp->af;

    /* ---------------------------------------------------------------
     * Step 1: Try full profile with v_peak = v_max
     * ------------------------------------------------------------- */
    if (compute_segments(prof, v0, vf, dp_abs, Vmax, A, J, dir)) {
        profile_integrate(prof);
        out->valid = true;
        return true;
    }

    /* ---------------------------------------------------------------
     * Step 2: Bisect on v_peak to find the highest achievable peak
     *         velocity (handles case where total distance is too short
     *         to reach v_max).
     * ------------------------------------------------------------- */
    fp_t v_lo = fp_max(v0, vf);
    fp_t v_hi = Vmax;

    /* Check that v_lo gives a valid (possibly zero-duration) profile */
    {
        /* Minimum distance to decelerate from v0 to vf: */
        fp_t d_min = ramp_distance(v0, vf, A, J, NULL);
        if (dp_abs < d_min - FP_EPS) {
            /* Cannot even decelerate in time — no valid profile */
            return false;
        }
    }

    /* Bisection: 40 iterations gives ~10^-12 relative precision, more
     * than enough for Q16.16 fixed-point. */
    for (int iter = 0; iter < 40; iter++) {
        fp_t v_mid = v_lo + fp_div(v_hi - v_lo, FP_TWO);
        if (compute_segments(prof, v0, vf, dp_abs, v_mid, A, J, dir)) {
            v_lo = v_mid;  /* feasible — try higher */
        } else {
            v_hi = v_mid;  /* infeasible — try lower */
        }
        if (v_hi - v_lo < FP_EPS) break;
    }

    /* Use the highest feasible v_peak found */
    if (compute_segments(prof, v0, vf, dp_abs, v_lo, A, J, dir)) {
        profile_integrate(prof);
        out->valid = true;
        return true;
    }

    /* Fallback: try with v_peak = max(v0, vf) (minimum possible peak) */
    fp_t v_peak_min = fp_max(v0, vf);
    if (v_peak_min < FP_EPS) v_peak_min = FP_EPS;
    if (compute_segments(prof, v0, vf, dp_abs, v_peak_min, A, J, dir)) {
        profile_integrate(prof);
        out->valid = true;
        return true;
    }

    /* No valid profile found */
    return false;
}
