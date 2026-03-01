/**
 * @file ruckig_fp.c
 * @brief Single-axis jerk-limited trajectory planner (fixed-point).
 *
 * ## Algorithm Overview
 *
 * We compute a "time-optimal" S-curve trajectory subject to velocity,
 * acceleration, and jerk limits.  The approach is the standard analytical
 * (closed-form) solution used in industrial motion controllers.
 *
 * ### Non-zero initial acceleration (a0)
 *
 * When `inp->a0 != 0` a single "zeroing" segment is prepended before the
 * standard 7-segment S-curve.  This segment applies jerk
 *   j_pre = -sign(a0) * j_max
 * for time
 *   t_pre = |a0| / j_max
 * which smoothly drives the acceleration to zero.  After the segment the
 * velocity has changed by
 *   Δv = sign(a0) * a0² / (2 * j_max)
 * and the position has shifted accordingly.  Planning then continues from
 * the new state (p_eff, v_eff, 0) to the original target.
 *
 * The total profile has up to 8 segments (1 pre + 7 main).
 *
 * ### Profile types (main 7-segment)
 *
 * Starting from the most-capable and falling back:
 *
 * 1. **Full 7-segment** — v_peak = v_max, both v_max and a_max reached.
 * 2. **Bisected v_peak** — v_max not reached; bisect to find optimal v_peak.
 * 3. **Fallback** — v_peak = max(v0, vf) (absolute minimum).
 *
 * ### Velocity mode
 *
 * When `inp->vel_mode == true` the planner sets `prof->vel_mode = true`.
 * After the trajectory ends at `(pf, vf, 0)` the profile extrapolates at
 * constant velocity `vf` (see `profile_at_time()`).  Quantisers never
 * report "finished" in this mode.
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
 * Compute the distance and duration of a symmetric jerk-limited ramp from
 * velocity v_start to velocity v_end (both ≥ 0, direction-normalised).
 *
 * The ramp is a 3-phase manoeuvre (jerk-ramp / const-accel / jerk-ramp)
 * that begins and ends with zero acceleration.  By symmetry the average
 * velocity is exactly (v_start + v_end) / 2:
 *
 *   distance = (v_start + v_end) / 2  ×  T_ramp
 *
 * where T_ramp is:
 *   - Triangular (|dv| ≤ A²/J):  T = 2 × sqrt(|dv| / J)
 *   - Trapezoidal (|dv| > A²/J):  T = A/J + |dv|/A
 *
 * @param v_start  Start velocity (≥ 0, direction-normalised).
 * @param v_end    End velocity (≥ 0, direction-normalised).
 * @param A        Acceleration limit (positive).
 * @param J        Jerk limit (positive).
 * @param out_dur  Output: ramp duration (may be NULL).
 * @return         Distance covered (always ≥ 0).
 */
static fp_t ramp_distance(fp_t v_start, fp_t v_end, fp_t A, fp_t J,
                           fp_t *out_dur)
{
    fp_t delta_v = fp_abs(v_end - v_start);
    /* A^2/J computed as A*(A/J) to avoid overflowing fp_t with A^2 directly.
     * e.g. A=500 → A*A=250000 which exceeds Q16.16 max ~32767, but A/J=0.25
     * is well within range, so fp_mul(A, fp_div(A,J)) stays in range. */
    fp_t A2_over_J = fp_mul(A, fp_div(A, J));
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
 * writing them into prof->t[] and prof->j[] starting at index `offset`.
 *
 * @param prof    Profile to write into.
 * @param offset  Starting index (0 for no pre-segment, 1 with pre-segment).
 * @param v0      Direction-normalised initial velocity (≥ 0).
 * @param vf      Direction-normalised final velocity (≥ 0).
 * @param dp      Displacement (positive, direction-normalised).
 * @param v_peak  Desired peak velocity.
 * @param A       Acceleration limit (positive).
 * @param J       Jerk limit (positive).
 * @param dir     Direction: +1 or -1.
 * @return        true if t_coast ≥ 0 (profile physically valid).
 */
static bool compute_segments(SCurveProfile *prof, int offset,
                              fp_t v0, fp_t vf, fp_t dp,
                              fp_t v_peak, fp_t A, fp_t J, int dir)
{
    fp_t dur_accel = FP_ZERO;
    fp_t dur_decel = FP_ZERO;

    fp_t d_accel = ramp_distance(v0, v_peak, A, J, &dur_accel);
    fp_t d_decel = ramp_distance(v_peak, vf, A, J, &dur_decel);

    fp_t d_coast = dp - d_accel - d_decel;

    if (d_coast < -FP_EPS) {
        return false;
    }
    if (d_coast < FP_ZERO) d_coast = FP_ZERO;

    fp_t t4 = (v_peak > FP_EPS) ? fp_div(d_coast, v_peak) : FP_ZERO;

    /* Decompose acceleration ramp into segments 1,2,3 */
    fp_t dv_accel = v_peak - v0;
    /* A^2/J: same overflow-safe computation as in ramp_distance() */
    fp_t A2_J = fp_mul(A, fp_div(A, J));
    fp_t t1, t2, t3;

    if (dv_accel <= A2_J + FP_EPS) {
        t1 = fp_sqrt(fp_div(fp_max(dv_accel, FP_ZERO), J));
        t2 = FP_ZERO;
        t3 = t1;
    } else {
        t1 = fp_div(A, J);
        t2 = fp_div(dv_accel, A) - t1;
        t3 = t1;
    }

    /* Decompose deceleration ramp into segments 5,6,7 */
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

    /* Sanity: all durations must be non-negative */
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

    /* Write segments at the requested offset (UDDU jerk pattern) */
    fp_t Jd = (dir > 0) ? J : -J;
    prof->t[offset+0] = t1; prof->j[offset+0] = +Jd;
    prof->t[offset+1] = t2; prof->j[offset+1] = FP_ZERO;
    prof->t[offset+2] = t3; prof->j[offset+2] = -Jd;
    prof->t[offset+3] = t4; prof->j[offset+3] = FP_ZERO;
    prof->t[offset+4] = t5; prof->j[offset+4] = -Jd;
    prof->t[offset+5] = t6; prof->j[offset+5] = FP_ZERO;
    prof->t[offset+6] = t7; prof->j[offset+6] = +Jd;

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

    fp_t J    = inp->j_max;
    fp_t A    = inp->a_max;
    fp_t Vmax = inp->v_max;

    /* ------------------------------------------------------------------
     * Pre-segment: if a0 != 0, prepend a jerk segment that drives the
     * acceleration to zero before the main 7-segment planner runs.
     *
     * Kinematics of the pre-segment (duration t_pre, jerk j_pre):
     *   j_pre  = -sign(a0) * J          (opposite sign to a0)
     *   t_pre  = |a0| / J               (time to reach a = 0)
     *   v_eff  = v0 + a0*t_pre + j_pre*t_pre^2/2
     *   p_eff  = p0 + v0*t_pre + a0*t_pre^2/2 + j_pre*t_pre^3/6
     * ---------------------------------------------------------------- */
    int  n_pre = 0;
    fp_t j_pre = FP_ZERO;
    fp_t t_pre = FP_ZERO;
    fp_t v_eff = inp->v0;   /* velocity at start of main plan */
    fp_t p_eff = inp->p0;   /* position at start of main plan */

    if (fp_abs(inp->a0) > FP_EPS) {
        fp_t a0_abs = fp_abs(inp->a0);
        j_pre = (inp->a0 > 0) ? -J : +J;
        t_pre = fp_div(a0_abs, J);

        fp_t t_pre2 = fp_mul(t_pre, t_pre);
        fp_t t_pre3 = fp_mul(t_pre2, t_pre);

        /* Effective state after the zeroing segment */
        v_eff = inp->v0
              + fp_mul(inp->a0, t_pre)
              + fp_div(fp_mul(j_pre, t_pre2), FP_TWO);
        p_eff = inp->p0
              + fp_mul(inp->v0, t_pre)
              + fp_div(fp_mul(inp->a0, t_pre2), FP_TWO)
              + fp_div(fp_mul(j_pre, t_pre3), FP_SIX);

        /* Safety: reject if zeroing a0 would violate velocity limits */
        if (v_eff > Vmax + FP_EPS || v_eff < -Vmax - FP_EPS) {
            return false;
        }

        n_pre = 1;
    }

    SCurveProfile *prof = &out->profile;

    /* Store the actual initial conditions (pre-segment uses these) */
    prof->a[0]    = inp->a0;
    prof->v[0]    = inp->v0;
    prof->p[0]    = inp->p0;
    prof->pf      = inp->pf;
    prof->vf      = inp->vf;
    prof->af      = inp->af;
    prof->vel_mode = inp->vel_mode;

    /* If a pre-segment exists, store it at index 0 */
    if (n_pre > 0) {
        prof->t[0] = t_pre;
        prof->j[0] = j_pre;
    }

    /* Displacement remaining for the main plan (after pre-segment) */
    fp_t dp = inp->pf - p_eff;

    /* ------------------------------------------------------------------
     * Trivial case: zero (or near-zero) remaining displacement
     * ---------------------------------------------------------------- */
    if (fp_abs(dp) <= FP_EPS && fp_abs(v_eff - inp->vf) <= FP_EPS) {
        /* Nothing left to do; fill 7 zero-duration main segments */
        int off = n_pre;
        for (int i = 0; i < SCURVE_SEGMENTS; i++) {
            prof->t[off + i] = FP_ZERO;
            prof->j[off + i] = FP_ZERO;
        }
        prof->n_segs = n_pre + SCURVE_SEGMENTS;
        profile_integrate(prof);
        out->valid = true;
        return true;
    }

    /* ------------------------------------------------------------------
     * Determine motion direction from the remaining displacement
     * ---------------------------------------------------------------- */
    int dir;
    if (dp > FP_EPS) {
        dir = 1;
    } else if (dp < -FP_EPS) {
        dir = -1;
    } else {
        /* dp ≈ 0 but velocities differ: fall through to main planner with
         * direction from the velocity difference */
        dir = (inp->vf >= v_eff) ? 1 : -1;
    }

    fp_t dp_abs = fp_abs(dp);

    /* Direction-normalised velocities for the main 7-segment solver */
    fp_t v0_plan = (dir > 0) ? v_eff : -v_eff;
    fp_t vf_plan = (dir > 0) ? inp->vf : -inp->vf;

    /* Clamp to [0, Vmax] */
    v0_plan = fp_max(FP_ZERO, fp_min(v0_plan, Vmax));
    vf_plan = fp_max(FP_ZERO, fp_min(vf_plan, Vmax));

    /* ------------------------------------------------------------------
     * Plan the main 7-segment S-curve at index n_pre..n_pre+6
     * ---------------------------------------------------------------- */
    bool ok = false;

    /* Try full profile with v_peak = v_max */
    if (compute_segments(prof, n_pre, v0_plan, vf_plan, dp_abs, Vmax, A, J, dir)) {
        ok = true;
    }

    if (!ok) {
        /* Check minimum feasible distance */
        fp_t d_min = ramp_distance(v0_plan, vf_plan, A, J, NULL);
        if (dp_abs < d_min - FP_EPS) {
            return false;
        }

        /* Bisect on v_peak to find the highest achievable peak velocity */
        fp_t v_lo = fp_max(v0_plan, vf_plan);
        fp_t v_hi = Vmax;

        for (int iter = 0; iter < 40; iter++) {
            fp_t v_mid = v_lo + fp_div(v_hi - v_lo, FP_TWO);
            if (compute_segments(prof, n_pre, v0_plan, vf_plan, dp_abs, v_mid, A, J, dir)) {
                v_lo = v_mid;
            } else {
                v_hi = v_mid;
            }
            if (v_hi - v_lo < FP_EPS) break;
        }

        if (compute_segments(prof, n_pre, v0_plan, vf_plan, dp_abs, v_lo, A, J, dir)) {
            ok = true;
        }

        /* Last-resort fallback */
        if (!ok) {
            fp_t v_peak_min = fp_max(v0_plan, vf_plan);
            if (v_peak_min < FP_EPS) v_peak_min = FP_EPS;
            ok = compute_segments(prof, n_pre, v0_plan, vf_plan, dp_abs,
                                  v_peak_min, A, J, dir);
        }
    }

    if (!ok) return false;

    prof->n_segs = n_pre + SCURVE_SEGMENTS;
    profile_integrate(prof);
    out->valid = true;
    return true;
}

/* -------------------------------------------------------------------------
 * ruckig_fp_replan
 * --------------------------------------------------------------------- */

bool ruckig_fp_replan(fp_t elapsed_time,
                      const RuckigFpOutput *current_out,
                      const RuckigFpInput  *new_inp,
                      RuckigFpOutput       *new_out)
{
    if (!current_out || !new_inp || !new_out) return false;
    if (!current_out->valid) return false;

    /* Sample the current trajectory to get (pos, vel, acc) now */
    fp_t cur_pos, cur_vel, cur_acc;
    profile_at_time(&current_out->profile, elapsed_time,
                    &cur_pos, &cur_vel, &cur_acc);

    /* Build new input: override start state with the sampled state */
    RuckigFpInput updated = *new_inp;
    updated.p0 = cur_pos;
    updated.v0 = cur_vel;
    updated.a0 = cur_acc;

    return ruckig_fp_calculate(&updated, new_out);
}
