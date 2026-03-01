/**
 * @file quantise.c
 * @brief Time-step and distance-step quantisation implementations.
 *
 * No <math.h> or floating-point arithmetic is used here.
 */

#include <stddef.h>
#include "ruckig_fp/quantise.h"

/* =========================================================================
 * Time-Step Quantiser
 * ======================================================================= */

void tsq_init(TimeStepQuantiser *q, const SCurveProfile *prof, fp_t dt)
{
    q->profile      = prof;
    q->current_time = FP_ZERO;
    q->dt           = dt;
    q->finished     = false;
}

bool tsq_step(TimeStepQuantiser *q, fp_t *pos, fp_t *vel, fp_t *acc)
{
    if (q->finished) {
        /* Return the final state when already finished (not vel_mode) */
        int n = q->profile->n_segs;
        if (pos) *pos = q->profile->p[n];
        if (vel) *vel = q->profile->v[n];
        if (acc) *acc = q->profile->a[n];
        return false;
    }

    fp_t total = profile_total_duration(q->profile);

    /* Evaluate the profile at the current time.
     * In vel_mode, profile_at_time() extrapolates beyond total. */
    profile_at_time(q->profile, q->current_time, pos, vel, acc);

    /* Advance the time cursor */
    q->current_time += q->dt;

    /* In velocity mode the trajectory never ends; always return true. */
    if (q->profile->vel_mode) {
        return true;
    }

    /* Mark finished once we have stepped past the end */
    if (q->current_time >= total) {
        q->current_time = total;
        q->finished = true;
    }

    return true;
}

/* =========================================================================
 * Distance-Step Quantiser
 * ======================================================================= */

void dsq_init(DistStepQuantiser *q, const SCurveProfile *prof, fp_t step_size)
{
    q->profile       = prof;
    q->step_size     = step_size;
    q->current_time  = FP_ZERO;
    q->finished      = false;

    /* The first target step is one step_size beyond the initial position */
    q->next_step_pos = prof->p[0] + step_size;
}

bool dsq_next_step_time(DistStepQuantiser *q, fp_t *step_time)
{
    if (q->finished) {
        return false;
    }

    fp_t total   = profile_total_duration(q->profile);
    int  n       = q->profile->n_segs;
    fp_t p_final = q->profile->p[n];

    /* ---------------------------------------------------------------
     * Is the next step position reachable?
     * In velocity mode the position grows without bound, so the step
     * is always reachable.  Without velocity mode, check the endpoint.
     * ------------------------------------------------------------- */
    if (!q->profile->vel_mode) {
        if (q->next_step_pos > p_final + (q->step_size >> 1)) {
            q->finished = true;
            return false;
        }
    }

    /* ---------------------------------------------------------------
     * Bisection search for t such that p(t) == next_step_pos.
     *
     * In velocity mode, extend the upper bound well beyond total so the
     * linear extrapolation can be reached.
     * ------------------------------------------------------------- */
    fp_t lo = q->current_time;
    fp_t hi;

    if (q->profile->vel_mode) {
        /* Estimate upper bound: time for linear extrapolation to reach pos */
        fp_t v_cruise = q->profile->v[n];
        if (q->next_step_pos <= p_final) {
            /* Step is within the planned profile — use normal upper bound */
            hi = total;
        } else if (fp_abs(v_cruise) > FP_ZERO) {
            /* Step is beyond the profile end; extrapolate using cruise speed.
             * If the axis is not moving toward the step (opposite sign), stop. */
            fp_t t_extra = fp_div(q->next_step_pos - p_final, v_cruise);
            if (t_extra < FP_ZERO) {
                q->finished = true;
                return false;
            }
            hi = total + fp_max(t_extra + FP_ONE, FP_ONE);
        } else {
            /* Axis stopped; no more steps possible */
            q->finished = true;
            return false;
        }
    } else {
        hi = total;
    }

    /* Verify the upper bound satisfies the invariant p(hi) >= next_step_pos */
    fp_t p_hi = FP_ZERO;
    profile_at_time(q->profile, hi, &p_hi, NULL, NULL);
    if (p_hi < q->next_step_pos - (q->step_size >> 1)) {
        q->finished = true;
        return false;
    }

    /* 32 iterations give sub-LSB precision for Q16.16 */
    for (int iter = 0; iter < 32; iter++) {
        fp_t mid = lo + fp_div(hi - lo, FP_TWO);
        fp_t p_mid = FP_ZERO;
        profile_at_time(q->profile, mid, &p_mid, NULL, NULL);
        if (p_mid < q->next_step_pos) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    if (step_time) *step_time = hi;

    q->current_time  = hi;
    q->next_step_pos += q->step_size;

    return true;
}
