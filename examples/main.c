/**
 * @file main.c
 * @brief Example program demonstrating ruckig_fp:
 *  1. Standard 0→1000 unit move (time-step and distance-step quantisers)
 *  2. Online replanning: change target mid-move
 *  3. Velocity mode: arrive at a position and continue at cruise speed
 *
 * Build with BUILD_EXAMPLES=ON:
 *   cmake -DBUILD_EXAMPLES=ON ..
 *   make
 *   ./ruckig_fp_example
 */

#include <stdio.h>
#include "ruckig_fp/ruckig_fp.h"
#include "ruckig_fp/quantise.h"

/* Common motion limits used throughout the examples */
static const fp_t V_MAX = FP_FROM_INT(200);   /* 200 units/s */
static const fp_t A_MAX = FP_FROM_INT(500);   /* 500 units/s^2 */
static const fp_t J_MAX = FP_FROM_INT(2000);  /* 2000 units/s^3 */

/* =========================================================================
 * Example 1 — Standard move with both quantiser modes
 * ======================================================================= */
static void example_standard(void)
{
    printf("=== Example 1: Standard 0 → 1000 unit move ===\n");

    RuckigFpInput inp = {
        .p0    = FP_FROM_INT(0),
        .v0    = FP_FROM_INT(0),
        .a0    = FP_FROM_INT(0),
        .pf    = FP_FROM_INT(1000),
        .vf    = FP_FROM_INT(0),
        .af    = FP_FROM_INT(0),
        .v_max = V_MAX,
        .v_min = -V_MAX,
        .a_max = A_MAX,
        .a_min = -A_MAX,
        .j_max = J_MAX,
        .vel_mode = false,
    };

    RuckigFpOutput out;
    if (!ruckig_fp_calculate(&inp, &out)) {
        printf("  ERROR: planning failed\n\n");
        return;
    }

    fp_t total_dur = profile_total_duration(&out.profile);
    int  n         = out.profile.n_segs;
    printf("  Total duration  : %.4f s\n",  FP_TO_FLOAT(total_dur));
    printf("  Segments used   : %d\n",      n);
    printf("  Final position  : %.4f units\n\n",
           FP_TO_FLOAT(out.profile.p[n]));

    /* --- Time-step quantiser (1 ms) --- */
    printf("  [Time-Step, dt=1ms, every 20th sample]\n");
    printf("  time(s)    pos(u)     vel(u/s)   acc(u/s^2)\n");
    printf("  --------   --------   --------   ----------\n");

    TimeStepQuantiser tsq;
    tsq_init(&tsq, &out.profile, FP_FROM_FLOAT(0.001f));

    fp_t pos, vel, acc;
    int ts_count = 0;
    while (tsq_step(&tsq, &pos, &vel, &acc)) {
        if (ts_count % 20 == 0) {
            printf("  %8.4f   %8.3f   %8.3f   %10.3f\n",
                   FP_TO_FLOAT(tsq.current_time - tsq.dt),
                   FP_TO_FLOAT(pos), FP_TO_FLOAT(vel), FP_TO_FLOAT(acc));
        }
        ts_count++;
    }
    printf("  (total %d time steps)\n\n", ts_count);

    /* --- Distance-step quantiser (1 unit/step) --- */
    printf("  [Distance-Step, step=1 unit, every 50th step]\n");
    printf("  step#   step_time(s)   interval(ms)\n");
    printf("  ------  ------------   ------------\n");

    DistStepQuantiser dsq;
    dsq_init(&dsq, &out.profile, FP_FROM_INT(1));

    fp_t step_time, prev_time = FP_ZERO;
    int step_count = 0;
    while (dsq_next_step_time(&dsq, &step_time)) {
        fp_t interval_ms = fp_mul(step_time - prev_time, FP_FROM_INT(1000));
        if (step_count % 50 == 0) {
            printf("  %6d  %12.6f   %12.4f\n",
                   step_count + 1, FP_TO_FLOAT(step_time),
                   FP_TO_FLOAT(interval_ms));
        }
        prev_time = step_time;
        step_count++;
    }
    printf("  (total %d steps)\n\n", step_count);
}

/* =========================================================================
 * Example 2 — Online replanning
 *
 * Plan a move 0 → 1000.  At t = 2.0 s (mid-coast phase) replan toward a
 * new target of 600 units (axis has overshot the desired stopping point).
 * The replan automatically picks up the current (pos, vel, acc) as its
 * initial conditions, including the non-zero acceleration that may exist at
 * the replanning instant.
 * ======================================================================= */
static void example_replan(void)
{
    printf("=== Example 2: Online Replanning ===\n");

    /* Initial plan: 0 → 1000 */
    RuckigFpInput inp = {
        .p0 = FP_FROM_INT(0),    .v0 = FP_FROM_INT(0), .a0 = FP_FROM_INT(0),
        .pf = FP_FROM_INT(1000), .vf = FP_FROM_INT(0), .af = FP_FROM_INT(0),
        .v_max = V_MAX, .v_min = -V_MAX,
        .a_max = A_MAX, .a_min = -A_MAX,
        .j_max = J_MAX, .vel_mode = false,
    };
    RuckigFpOutput out1;
    if (!ruckig_fp_calculate(&inp, &out1)) {
        printf("  ERROR: initial plan failed\n\n");
        return;
    }

    /* Sample the state at t = 0.3 s (mid-acceleration, non-zero acc) */
    fp_t t_replan = FP_FROM_FLOAT(0.3f);
    fp_t snap_pos, snap_vel, snap_acc;
    profile_at_time(&out1.profile, t_replan, &snap_pos, &snap_vel, &snap_acc);
    printf("  State at t=0.3 s (during acceleration, a0 != 0):\n");
    printf("    pos=%.3f  vel=%.3f  acc=%.3f\n\n",
           FP_TO_FLOAT(snap_pos), FP_TO_FLOAT(snap_vel), FP_TO_FLOAT(snap_acc));

    /* Replan toward new target 500 with non-zero a0 captured automatically */
    RuckigFpInput new_inp = inp;
    new_inp.pf = FP_FROM_INT(500);

    RuckigFpOutput out2;
    if (!ruckig_fp_replan(t_replan, &out1, &new_inp, &out2)) {
        printf("  ERROR: replan failed\n\n");
        return;
    }

    fp_t new_dur = profile_total_duration(&out2.profile);
    int  n2      = out2.profile.n_segs;
    printf("  Replanned trajectory (target = 500, starting with a0=%.1f):\n",
           FP_TO_FLOAT(snap_acc));
    printf("    Segments  : %d  (8 = 1 pre-segment for a0 + 7 S-curve)\n", n2);
    printf("    Duration  : %.4f s\n", FP_TO_FLOAT(new_dur));
    printf("    Final pos : %.4f units\n\n",
           FP_TO_FLOAT(out2.profile.p[n2]));

    /* Print first 15 time-step samples of the replanned trajectory */
    printf("  [Replanned, dt=1ms, first 15 samples]\n");
    printf("  time(s)    pos(u)     vel(u/s)   acc(u/s^2)\n");
    printf("  --------   --------   --------   ----------\n");

    TimeStepQuantiser tsq;
    tsq_init(&tsq, &out2.profile, FP_FROM_FLOAT(0.001f));
    fp_t pos, vel, acc;
    int cnt = 0;
    while (tsq_step(&tsq, &pos, &vel, &acc) && cnt < 15) {
        printf("  %8.4f   %8.3f   %8.3f   %10.3f\n",
               FP_TO_FLOAT(tsq.current_time - tsq.dt),
               FP_TO_FLOAT(pos), FP_TO_FLOAT(vel), FP_TO_FLOAT(acc));
        cnt++;
    }
    printf("  ...\n\n");
}

/* =========================================================================
 * Example 3 — Velocity mode
 *
 * Move from 0 to position 300 arriving at 100 units/s, then continue at
 * that speed indefinitely (velocity mode).  The time-step quantiser runs
 * for 40 ms beyond the planned trajectory end to demonstrate the constant-
 * velocity cruise, and the distance-step quantiser finds an extra 10 steps
 * beyond the target.
 * ======================================================================= */
static void example_vel_mode(void)
{
    printf("=== Example 3: Velocity Mode ===\n");

    RuckigFpInput inp = {
        .p0 = FP_FROM_INT(0),   .v0 = FP_FROM_INT(0), .a0 = FP_FROM_INT(0),
        .pf = FP_FROM_INT(300), .vf = FP_FROM_INT(100),.af = FP_FROM_INT(0),
        .v_max = V_MAX, .v_min = -V_MAX,
        .a_max = A_MAX, .a_min = -A_MAX,
        .j_max = J_MAX,
        .vel_mode = true,   /* continue at vf = 100 units/s after p = 300 */
    };

    RuckigFpOutput out;
    if (!ruckig_fp_calculate(&inp, &out)) {
        printf("  ERROR: planning failed\n\n");
        return;
    }

    fp_t total_dur = profile_total_duration(&out.profile);
    int  n         = out.profile.n_segs;
    printf("  Profile duration : %.4f s  (cruise starts after this)\n",
           FP_TO_FLOAT(total_dur));
    printf("  Position at end  : %.4f units\n",
           FP_TO_FLOAT(out.profile.p[n]));
    printf("  Velocity at end  : %.4f units/s  (cruise speed)\n\n",
           FP_TO_FLOAT(out.profile.v[n]));

    /* Time-step quantiser: show samples across the transition into cruise */
    printf("  [Vel-mode time-step, dt=10ms, crossing profile end]\n");
    printf("  time(s)    pos(u)     vel(u/s)   acc(u/s^2)   in_cruise\n");
    printf("  --------   --------   --------   ----------   ---------\n");

    TimeStepQuantiser tsq;
    fp_t dt = FP_FROM_FLOAT(0.010f);  /* 10 ms steps */
    tsq_init(&tsq, &out.profile, dt);

    fp_t pos, vel, acc;
    int ts_count = 0;
    /* Run for 20 steps beyond the planned profile end */
    fp_t stop_time = total_dur + FP_FROM_FLOAT(0.20f);
    while (tsq_step(&tsq, &pos, &vel, &acc)) {
        fp_t t_now = tsq.current_time - tsq.dt;
        int  cruise = (t_now >= total_dur) ? 1 : 0;
        printf("  %8.4f   %8.3f   %8.3f   %10.3f   %s\n",
               FP_TO_FLOAT(t_now), FP_TO_FLOAT(pos),
               FP_TO_FLOAT(vel),   FP_TO_FLOAT(acc),
               cruise ? "YES" : "no");
        ts_count++;
        if (tsq.current_time > stop_time) break;  /* manual stop */
    }
    printf("  (stopped after %d steps; quantiser would continue indefinitely)\n\n",
           ts_count);

    /* Distance-step quantiser: show steps beyond the target position */
    printf("  [Vel-mode distance-step, step=1 unit, steps 295..310]\n");
    printf("  step#   step_time(s)   interval(ms)\n");
    printf("  ------  ------------   ------------\n");

    DistStepQuantiser dsq;
    dsq_init(&dsq, &out.profile, FP_FROM_INT(1));

    fp_t step_time, prev_time = FP_ZERO;
    int step_count = 0;
    while (dsq_next_step_time(&dsq, &step_time)) {
        fp_t interval_ms = fp_mul(step_time - prev_time, FP_FROM_INT(1000));
        if (step_count >= 294 && step_count <= 310) {
            printf("  %6d  %12.6f   %12.4f\n",
                   step_count + 1, FP_TO_FLOAT(step_time),
                   FP_TO_FLOAT(interval_ms));
        }
        prev_time  = step_time;
        step_count++;
        if (step_count > 312) break;  /* manual stop */
    }
    printf("  (stopped at step %d; quantiser would continue indefinitely)\n\n",
           step_count);
}

/* =========================================================================
 * main
 * ======================================================================= */

int main(void)
{
    example_standard();
    example_replan();
    example_vel_mode();
    return 0;
}
