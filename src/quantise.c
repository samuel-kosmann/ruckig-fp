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
        /* Return the final state when already finished */
        if (pos) *pos = q->profile->p[SCURVE_SEGMENTS];
        if (vel) *vel = q->profile->v[SCURVE_SEGMENTS];
        if (acc) *acc = q->profile->a[SCURVE_SEGMENTS];
        return false;
    }

    fp_t total = profile_total_duration(q->profile);

    /* Evaluate the profile at the current time */
    profile_at_time(q->profile, q->current_time, pos, vel, acc);

    /* Advance the time cursor */
    q->current_time += q->dt;

    /* Check for completion */
    if (q->current_time >= total) {
        q->current_time = total;
        q->finished = true;
        /* Return true for the last valid sample, but mark finished so the
         * next call returns false.  We already evaluated above, so return
         * true this time (caller gets this sample). */
        return true;
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

    fp_t total = profile_total_duration(q->profile);

    /* Check whether the next step position is reachable at all */
    fp_t p_final = q->profile->p[SCURVE_SEGMENTS];
    if (q->next_step_pos > p_final + (q->step_size >> 1)) {
        q->finished = true;
        return false;
    }

    /* ------------------------------------------------------------------
     * Bisection search for t such that position(t) == next_step_pos.
     *
     * We search in the interval [current_time, total_duration].
     * Invariant: p(lo) < next_step_pos <= p(hi).
     * ---------------------------------------------------------------- */
    fp_t lo = q->current_time;
    fp_t hi = total;

    /* Verify the upper bound can satisfy the invariant */
    fp_t p_hi = FP_ZERO;
    profile_at_time(q->profile, hi, &p_hi, NULL, NULL);
    if (p_hi < q->next_step_pos - (q->step_size >> 1)) {
        q->finished = true;
        return false;
    }

    /* 32 iterations: each halves the interval, giving sub-LSB precision */
    for (int iter = 0; iter < 32; iter++) {
        /* mid = (lo + hi) / 2  without overflow */
        fp_t mid = lo + fp_div(hi - lo, FP_TWO);

        fp_t p_mid = FP_ZERO;
        profile_at_time(q->profile, mid, &p_mid, NULL, NULL);

        if (p_mid < q->next_step_pos) {
            lo = mid;   /* step not yet reached at mid */
        } else {
            hi = mid;   /* step already reached at mid */
        }
    }

    /* hi is the first time position >= next_step_pos */
    if (step_time) *step_time = hi;

    /* Advance state for the next call */
    q->current_time  = hi;
    q->next_step_pos += q->step_size;

    return true;
}
