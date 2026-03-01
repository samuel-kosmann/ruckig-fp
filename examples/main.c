/**
 * @file main.c
 * @brief Example program demonstrating ruckig_fp time-step and distance-step
 *        quantisation for a 0→1000 unit move.
 *
 * Build with BUILD_EXAMPLES=ON:
 *   cmake -DBUILD_EXAMPLES=ON ..
 *   make
 *   ./ruckig_fp_example
 *
 * Expected output: a table of time/position/velocity/acceleration samples
 * followed by a list of step-pulse times for the same trajectory.
 */

#include <stdio.h>
#include "ruckig_fp/ruckig_fp.h"
#include "ruckig_fp/quantise.h"

int main(void)
{
    /* ------------------------------------------------------------------
     * Define a 0 → 1000 unit move at modest limits.
     * ---------------------------------------------------------------- */
    RuckigFpInput inp = {
        .p0    = FP_FROM_INT(0),
        .v0    = FP_FROM_INT(0),
        .a0    = FP_FROM_INT(0),
        .pf    = FP_FROM_INT(1000),
        .vf    = FP_FROM_INT(0),
        .af    = FP_FROM_INT(0),
        .v_max = FP_FROM_INT(200),    /* 200 units/s */
        .v_min = FP_FROM_INT(-200),
        .a_max = FP_FROM_INT(500),    /* 500 units/s^2 */
        .a_min = FP_FROM_INT(-500),
        .j_max = FP_FROM_INT(2000),   /* 2000 units/s^3 */
    };

    RuckigFpOutput out;
    if (!ruckig_fp_calculate(&inp, &out)) {
        printf("ERROR: ruckig_fp_calculate failed\n");
        return 1;
    }

    fp_t total_dur = profile_total_duration(&out.profile);
    printf("Trajectory planned successfully.\n");
    printf("Total duration: %.4f s\n", FP_TO_FLOAT(total_dur));
    printf("Final position: %.4f units\n",
           FP_TO_FLOAT(out.profile.p[SCURVE_SEGMENTS]));
    printf("\n");

    /* ------------------------------------------------------------------
     * Time-step quantisation (1 ms steps) — DC motor usage
     * ---------------------------------------------------------------- */
    printf("=== Time-Step Quantiser (dt = 1 ms) ===\n");
    printf("  time(s)    pos(u)     vel(u/s)   acc(u/s^2)\n");
    printf("  --------   --------   --------   ----------\n");

    TimeStepQuantiser tsq;
    tsq_init(&tsq, &out.profile, FP_FROM_FLOAT(0.001f)); /* 1 ms */

    fp_t pos, vel, acc;
    int ts_count = 0;
    while (tsq_step(&tsq, &pos, &vel, &acc)) {
        /* Print every 20th sample to keep output manageable */
        if (ts_count % 20 == 0) {
            printf("  %8.4f   %8.3f   %8.3f   %10.3f\n",
                   FP_TO_FLOAT(tsq.current_time - tsq.dt),
                   FP_TO_FLOAT(pos),
                   FP_TO_FLOAT(vel),
                   FP_TO_FLOAT(acc));
        }
        ts_count++;
    }
    printf("  (total %d time steps)\n\n", ts_count);

    /* ------------------------------------------------------------------
     * Distance-step quantisation (1 unit per step) — stepper motor usage
     * ---------------------------------------------------------------- */
    printf("=== Distance-Step Quantiser (step_size = 1 unit) ===\n");
    printf("  step#   step_time(s)   interval(ms)\n");
    printf("  ------  ------------   ------------\n");

    DistStepQuantiser dsq;
    dsq_init(&dsq, &out.profile, FP_FROM_INT(1)); /* 1 unit/step */

    fp_t step_time;
    fp_t prev_time = FP_ZERO;
    int step_count = 0;
    while (dsq_next_step_time(&dsq, &step_time)) {
        fp_t interval_ms = fp_mul(step_time - prev_time, FP_FROM_INT(1000));
        /* Print every 50th step */
        if (step_count % 50 == 0) {
            printf("  %6d  %12.6f   %12.4f\n",
                   step_count + 1,
                   FP_TO_FLOAT(step_time),
                   FP_TO_FLOAT(interval_ms));
        }
        prev_time = step_time;
        step_count++;
    }
    printf("  (total %d steps)\n", step_count);

    return 0;
}
