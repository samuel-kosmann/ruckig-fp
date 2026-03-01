/**
 * @file main.cpp
 * @brief Interactive ruckig_fp S-curve visualiser.
 *
 * Build with BUILD_IMGUI_DEMO=ON (requires GLFW + OpenGL 3.x):
 *
 *   cmake -DBUILD_IMGUI_DEMO=ON ..
 *   make ruckig_fp_imgui_demo
 *   ./ruckig_fp_imgui_demo
 *
 * Controls:
 *   Left panel — sliders for start state (p0, v0, a0), target state
 *                (pf, vf), and motion limits (v_max, a_max, j_max).
 *   Right panel — four stacked live plots: position, velocity,
 *                 acceleration, and jerk vs. time.
 *
 * The trajectory is replanned on every UI frame, giving instant feedback.
 */

/*
 * IMPORTANT include order:
 * glibc's <math.h> defines FP_ZERO as an enum member constant (= 2).
 * ruckig_fp's fixed_point.h defines FP_ZERO as ((fp_t)0).
 * If our macro is in scope when <math.h> is included, the enum breaks.
 * Solution: include all system/third-party headers first, then #undef
 * glibc's FP_ZERO, then include ruckig_fp headers.
 */

/* --- System / STL (must come before ruckig_fp) --- */
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>

/* --- OpenGL / GLFW --- */
#include <GLFW/glfw3.h>

/* --- Dear ImGui --- */
#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>

/* --- ImPlot (vendored) --- */
#include "implot/implot.h"

/*
 * After <cmath>, FP_ZERO is 2 (enum value, no longer a macro).
 * Undef it so fixed_point.h can define FP_ZERO = ((fp_t)0).
 */
#undef FP_ZERO
#include "ruckig_fp/ruckig_fp.h"

/* =========================================================================
 * Constants
 * ======================================================================= */

static const int  PLOT_SAMPLES = 800;
static const char GLSL_VERSION[] = "#version 130";

/* =========================================================================
 * GLFW error callback
 * ======================================================================= */

static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

/* =========================================================================
 * Trajectory data
 * ======================================================================= */

struct TrajectoryData {
    std::vector<double> time;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> acc;
    std::vector<double> jerk;
    bool valid = false;
    double total_duration = 0.0;
};

/**
 * Plan a trajectory and sample it into PLOT_SAMPLES data points.
 * All planning is fixed-point; only the display output is float/double.
 */
static TrajectoryData compute_trajectory(
    float p0, float v0, float a0,
    float pf, float vf,
    float v_max, float a_max, float j_max)
{
    TrajectoryData td;

    RuckigFpInput inp;
    inp.p0       = FP_FROM_FLOAT(p0);
    inp.v0       = FP_FROM_FLOAT(v0);
    inp.a0       = FP_FROM_FLOAT(a0);
    inp.pf       = FP_FROM_FLOAT(pf);
    inp.vf       = FP_FROM_FLOAT(vf);
    inp.af       = FP_ZERO;
    inp.v_max    = FP_FROM_FLOAT(v_max);
    inp.v_min    = FP_FROM_FLOAT(-v_max);
    inp.a_max    = FP_FROM_FLOAT(a_max);
    inp.a_min    = FP_FROM_FLOAT(-a_max);
    inp.j_max    = FP_FROM_FLOAT(j_max);
    inp.vel_mode = false;

    RuckigFpOutput out;
    if (!ruckig_fp_calculate(&inp, &out)) {
        td.valid = false;
        return td;
    }

    td.valid = true;
    fp_t total_fp     = profile_total_duration(&out.profile);
    td.total_duration = (double)FP_TO_FLOAT(total_fp);

    /* Extend a little past the end so the final state is visible */
    double tail  = td.total_duration * 0.08 + 0.05;
    double t_end = td.total_duration + tail;

    td.time.resize(PLOT_SAMPLES);
    td.pos .resize(PLOT_SAMPLES);
    td.vel .resize(PLOT_SAMPLES);
    td.acc .resize(PLOT_SAMPLES);
    td.jerk.resize(PLOT_SAMPLES);

    for (int i = 0; i < PLOT_SAMPLES; ++i) {
        double t     = t_end * i / (PLOT_SAMPLES - 1);
        fp_t   fp_tv = FP_FROM_FLOAT((float)t);

        fp_t fp_pos, fp_vel, fp_acc, fp_acc_next;
        profile_at_time(&out.profile, fp_tv, &fp_pos, &fp_vel, &fp_acc);

        /* Jerk = finite-difference of acceleration over 0.1 ms */
        fp_t fp_dt = FP_FROM_FLOAT(1e-4f);
        profile_at_time(&out.profile, fp_tv + fp_dt, NULL, NULL, &fp_acc_next);
        double j = ((double)FP_TO_FLOAT(fp_acc_next)
                  - (double)FP_TO_FLOAT(fp_acc))
                  / (double)FP_TO_FLOAT(fp_dt);

        td.time[i] = t;
        td.pos [i] = (double)FP_TO_FLOAT(fp_pos);
        td.vel [i] = (double)FP_TO_FLOAT(fp_vel);
        td.acc [i] = (double)FP_TO_FLOAT(fp_acc);
        td.jerk[i] = j;
    }

    return td;
}

/* =========================================================================
 * main
 * ======================================================================= */

int main(void)
{
    /* --- GLFW init --- */
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialise GLFW\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(
        1280, 820, "ruckig_fp — Interactive S-curve Planner", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    /* --- ImGui + ImPlot init --- */
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(GLSL_VERSION);

    /* --- Parameter state --- */
    float p0    =   0.0f;
    float v0    =   0.0f;
    float a0    =   0.0f;
    float pf    = 100.0f;
    float vf    =   0.0f;
    float v_max =  50.0f;
    float a_max = 100.0f;
    float j_max = 300.0f;

    TrajectoryData td =
        compute_trajectory(p0, v0, a0, pf, vf, v_max, a_max, j_max);

    /* --- Main loop --- */
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        /* Recompute trajectory each frame */
        td = compute_trajectory(p0, v0, a0, pf, vf, v_max, a_max, j_max);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport *vp      = ImGui::GetMainViewport();
        const float    W       = vp->Size.x;
        const float    H       = vp->Size.y;
        const float    CTRL_W  = 300.0f;
        const float    PLOT_W  = W - CTRL_W;

        /* ================================================================
         * LEFT PANEL — sliders
         * ============================================================== */
        ImGui::SetNextWindowPos(vp->Pos);
        ImGui::SetNextWindowSize(ImVec2(CTRL_W, H));
        ImGui::Begin("Controls", nullptr,
            ImGuiWindowFlags_NoMove    | ImGuiWindowFlags_NoResize  |
            ImGuiWindowFlags_NoCollapse| ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f,0.9f,0.3f,1.0f));
        ImGui::TextUnformatted("ruckig_fp  S-Curve Planner");
        ImGui::PopStyleColor();
        ImGui::Separator(); ImGui::Spacing();

        /* Start state */
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f,0.9f,1.0f,1.0f));
        ImGui::TextUnformatted("Start State");
        ImGui::PopStyleColor();
        ImGui::PushItemWidth(-1);
        ImGui::SliderFloat("##p0", &p0, -200.0f, 200.0f, "p0 = %.1f");
        ImGui::SliderFloat("##v0", &v0,   -v_max, v_max, "v0 = %.2f");
        ImGui::SliderFloat("##a0", &a0,   -a_max, a_max, "a0 = %.2f");
        ImGui::PopItemWidth();
        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        /* Target state */
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f,1.0f,0.6f,1.0f));
        ImGui::TextUnformatted("Target State");
        ImGui::PopStyleColor();
        ImGui::PushItemWidth(-1);
        ImGui::SliderFloat("##pf", &pf, -200.0f, 200.0f, "pf = %.1f");
        ImGui::SliderFloat("##vf", &vf,   -v_max, v_max, "vf = %.2f");
        ImGui::PopItemWidth();
        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        /* Motion limits */
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f,0.7f,0.4f,1.0f));
        ImGui::TextUnformatted("Motion Limits");
        ImGui::PopStyleColor();
        ImGui::PushItemWidth(-1);
        ImGui::SliderFloat("##vm", &v_max,   1.0f,  200.0f, "v_max = %.1f");
        ImGui::SliderFloat("##am", &a_max,   1.0f,  500.0f, "a_max = %.1f");
        ImGui::SliderFloat("##jm", &j_max,   1.0f, 2000.0f, "j_max = %.1f");
        ImGui::PopItemWidth();

        /* Keep velocity/accel sliders within limits */
        v0 = std::max(-v_max, std::min(v_max, v0));
        vf = std::max(-v_max, std::min(v_max, vf));
        a0 = std::max(-a_max, std::min(a_max, a0));

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        /* Status */
        if (td.valid) {
            ImGui::TextColored(ImVec4(0.3f,1.0f,0.3f,1.0f), "Valid trajectory");
            ImGui::Text("Duration : %.4f s",  td.total_duration);
            ImGui::Text("Displ.   : %.2f u", (double)(pf - p0));
        } else {
            ImGui::TextColored(ImVec4(1.0f,0.4f,0.4f,1.0f), "No valid trajectory");
            ImGui::TextWrapped(
                "Check limits and initial conditions.\n"
                "(Try increasing j_max or reducing v0/a0)");
        }
        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        ImGui::TextDisabled("Yellow line = trajectory end");
        ImGui::TextDisabled("Maths: Q16.16 fixed-point");

        ImGui::End();

        /* ================================================================
         * RIGHT PANEL — four stacked plots
         * ============================================================== */
        ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + CTRL_W, vp->Pos.y));
        ImGui::SetNextWindowSize(ImVec2(PLOT_W, H));
        ImGui::Begin("Plots", nullptr,
            ImGuiWindowFlags_NoMove    | ImGuiWindowFlags_NoResize  |
            ImGuiWindowFlags_NoCollapse| ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoBringToFrontOnFocus);

        const float avail_h = ImGui::GetContentRegionAvail().y;
        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float ph      = avail_h / 4.0f - 4.0f;
        const double *tp    = td.valid ? td.time.data() : nullptr;
        const int     np    = td.valid ? PLOT_SAMPLES   : 0;

        auto end_line = [&]() {
            if (!td.valid || td.total_duration <= 0.0) return;
            double ref = td.total_duration;
            ImPlot::DragLineX(99, &ref,
                ImVec4(1.0f,1.0f,0.0f,0.7f), 1.5f,
                ImPlotDragToolFlags_NoInputs);
        };

        /* Position */
        if (ImPlot::BeginPlot("Position##p", ImVec2(avail_w, ph),
                ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("t (s)", "pos (u)");
            ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0.0, 1e6);
            ImPlot::SetNextLineStyle(ImVec4(0.4f,0.8f,1.0f,1.0f), 1.5f);
            if (td.valid) ImPlot::PlotLine("pos", tp, td.pos.data(), np);
            end_line();
            ImPlot::EndPlot();
        }

        /* Velocity */
        if (ImPlot::BeginPlot("Velocity##v", ImVec2(avail_w, ph),
                ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("t (s)", "vel (u/s)");
            ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0.0, 1e6);
            ImPlot::SetNextLineStyle(ImVec4(0.4f,1.0f,0.5f,1.0f), 1.5f);
            if (td.valid) ImPlot::PlotLine("vel", tp, td.vel.data(), np);
            end_line();
            ImPlot::EndPlot();
        }

        /* Acceleration */
        if (ImPlot::BeginPlot("Acceleration##a", ImVec2(avail_w, ph),
                ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("t (s)", "acc (u/s\xc2\xb2)");
            ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0.0, 1e6);
            ImPlot::SetNextLineStyle(ImVec4(1.0f,0.8f,0.3f,1.0f), 1.5f);
            if (td.valid) ImPlot::PlotLine("acc", tp, td.acc.data(), np);
            end_line();
            ImPlot::EndPlot();
        }

        /* Jerk */
        if (ImPlot::BeginPlot("Jerk##j", ImVec2(avail_w, ph),
                ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("t (s)", "jerk (u/s\xc2\xb3)");
            ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0.0, 1e6);
            ImPlot::SetNextLineStyle(ImVec4(1.0f,0.4f,0.4f,1.0f), 1.5f);
            if (td.valid) ImPlot::PlotLine("jerk", tp, td.jerk.data(), np);
            end_line();
            ImPlot::EndPlot();
        }

        ImGui::End();

        /* --- render --- */
        ImGui::Render();
        int fb_w, fb_h;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    /* --- cleanup --- */
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
