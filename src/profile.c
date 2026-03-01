/**
 * @file profile.c
 * @brief S-curve profile integration and evaluation.
 *
 * Implements the three public functions declared in profile.h.
 * No <math.h> or floating-point arithmetic is used here.
 */

#include "ruckig_fp/profile.h"

/* -------------------------------------------------------------------------
 * profile_total_duration
 * --------------------------------------------------------------------- */

fp_t profile_total_duration(const SCurveProfile *prof)
{
    fp_t total = FP_ZERO;
    for (int i = 0; i < SCURVE_SEGMENTS; i++) {
        total += prof->t[i];
    }
    return total;
}

/* -------------------------------------------------------------------------
 * profile_integrate
 * --------------------------------------------------------------------- */

/**
 * Kinematic update for one segment:
 *
 *   a_end = a0 + dt * j
 *   v_end = v0 + dt*a0 + dt^2*j / 2
 *   p_end = p0 + dt*v0 + dt^2*a0/2 + dt^3*j/6
 *
 * All computed in Q16.16 fixed-point using fp_mul / fp_div.
 */
void profile_integrate(SCurveProfile *prof)
{
    for (int i = 0; i < SCURVE_SEGMENTS; i++) {
        fp_t dt  = prof->t[i];
        fp_t j   = prof->j[i];
        fp_t a0  = prof->a[i];
        fp_t v0  = prof->v[i];
        fp_t p0  = prof->p[i];

        // dt^2 and dt^3 (fixed-point)
        fp_t dt2 = fp_mul(dt, dt);    // dt^2
        fp_t dt3 = fp_mul(dt2, dt);   // dt^3

        // a[i+1] = a0 + dt*j
        prof->a[i + 1] = a0 + fp_mul(dt, j);

        // v[i+1] = v0 + dt*a0 + dt^2*j/2
        prof->v[i + 1] = v0
                       + fp_mul(dt, a0)
                       + fp_div(fp_mul(dt2, j), FP_TWO);

        // p[i+1] = p0 + dt*v0 + dt^2*a0/2 + dt^3*j/6
        prof->p[i + 1] = p0
                       + fp_mul(dt, v0)
                       + fp_div(fp_mul(dt2, a0), FP_TWO)
                       + fp_div(fp_mul(dt3, j), FP_SIX);
    }
}

/* -------------------------------------------------------------------------
 * profile_at_time
 * --------------------------------------------------------------------- */

void profile_at_time(const SCurveProfile *prof, fp_t t_query,
                     fp_t *pos, fp_t *vel, fp_t *acc)
{
    fp_t total = profile_total_duration(prof);

    // Clamp query time to the valid range.
    if (t_query <= FP_ZERO) {
        if (pos) *pos = prof->p[0];
        if (vel) *vel = prof->v[0];
        if (acc) *acc = prof->a[0];
        return;
    }
    if (t_query >= total) {
        if (pos) *pos = prof->p[SCURVE_SEGMENTS];
        if (vel) *vel = prof->v[SCURVE_SEGMENTS];
        if (acc) *acc = prof->a[SCURVE_SEGMENTS];
        return;
    }

    // Walk segments to find which one contains t_query.
    fp_t t_seg_start = FP_ZERO;
    for (int i = 0; i < SCURVE_SEGMENTS; i++) {
        fp_t t_seg_end = t_seg_start + prof->t[i];

        if (t_query <= t_seg_end || i == SCURVE_SEGMENTS - 1) {
            // t_query is within segment i.
            // Local time dt within the segment.
            fp_t dt  = t_query - t_seg_start;
            fp_t j   = prof->j[i];
            fp_t a0  = prof->a[i];
            fp_t v0  = prof->v[i];
            fp_t p0  = prof->p[i];

            fp_t dt2 = fp_mul(dt, dt);
            fp_t dt3 = fp_mul(dt2, dt);

            if (acc) *acc = a0 + fp_mul(dt, j);
            if (vel) *vel = v0 + fp_mul(dt, a0) + fp_div(fp_mul(dt2, j), FP_TWO);
            if (pos) *pos = p0 + fp_mul(dt, v0)
                               + fp_div(fp_mul(dt2, a0), FP_TWO)
                               + fp_div(fp_mul(dt3, j), FP_SIX);
            return;
        }

        t_seg_start = t_seg_end;
    }
}
