/*
 * ASV Spline Tracking - Closed-Loop Simulation in C
 * Replicates the Python closed-loop MPC simulation using generated ACADOS solvers
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

// ACADOS includes
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_solver_asv_spline_tracking.h"
#include "acados_sim_solver_asv_spline_tracking.h"

// BLASFEO
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     ASV_SPLINE_TRACKING_NX
#define NU     ASV_SPLINE_TRACKING_NU
#define NP     ASV_SPLINE_TRACKING_NP
#define NBX0   ASV_SPLINE_TRACKING_NBX0
#define N_HORIZON ASV_SPLINE_TRACKING_N

// Simulation parameters
#define T_SIM 10.0      // Total simulation time [s]
#define TF 3.0          // MPC prediction horizon [s]
#define DT (TF / N_HORIZON)  // Time step

// Helper function to compute distance between points
double point_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx*dx + dy*dy);
}

// Get Catmull-Rom spline coefficients
void get_catmull_rom_segment(double p0[2], double p1[2], double p2[2], double p3[2], 
                              double spline_params[8]) {
    double alpha = 1.0;
    double tension = 0.2;
    
    // Compute segment lengths
    double t01 = pow(point_distance(p0[0], p0[1], p1[0], p1[1]), alpha);
    double t12 = pow(point_distance(p1[0], p1[1], p2[0], p2[1]), alpha);
    double t23 = pow(point_distance(p2[0], p2[1], p3[0], p3[1]), alpha);
    
    // Tangent vectors (x and y components)
    double m1_x = (1.0 - tension) * (p2[0] - p1[0] + t12 * ((p1[0] - p0[0]) / t01 - (p2[0] - p0[0]) / (t01 + t12)));
    double m1_y = (1.0 - tension) * (p2[1] - p1[1] + t12 * ((p1[1] - p0[1]) / t01 - (p2[1] - p0[1]) / (t01 + t12)));
    
    double m2_x = (1.0 - tension) * (p2[0] - p1[0] + t12 * ((p3[0] - p2[0]) / t23 - (p3[0] - p1[0]) / (t12 + t23)));
    double m2_y = (1.0 - tension) * (p2[1] - p1[1] + t12 * ((p3[1] - p2[1]) / t23 - (p3[1] - p1[1]) / (t12 + t23)));
    
    // Hermite basis coefficients
    double a_x = 2.0 * (p1[0] - p2[0]) + m1_x + m2_x;
    double b_x = -3.0 * (p1[0] - p2[0]) - m1_x - m1_x - m2_x;
    double c_x = m1_x;
    double d_x = p1[0];
    
    double a_y = 2.0 * (p1[1] - p2[1]) + m1_y + m2_y;
    double b_y = -3.0 * (p1[1] - p2[1]) - m1_y - m1_y - m2_y;
    double c_y = m1_y;
    double d_y = p1[1];
    
    // Pack into array [a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y]
    spline_params[0] = a_x;
    spline_params[1] = b_x;
    spline_params[2] = c_x;
    spline_params[3] = d_x;
    spline_params[4] = a_y;
    spline_params[5] = b_y;
    spline_params[6] = c_y;
    spline_params[7] = d_y;
}

// Evaluate spline at parameter t
void evaluate_spline(double t, double spline_params[8], double result[2]) {
    double t2 = t * t;
    double t3 = t2 * t;
    
    result[0] = spline_params[0] * t3 + spline_params[1] * t2 + spline_params[2] * t + spline_params[3];
    result[1] = spline_params[4] * t3 + spline_params[5] * t2 + spline_params[6] * t + spline_params[7];
}

// Save trajectory to CSV file
void save_trajectory_csv(const char* filename, double** simX, double** simU, int nsim) {
    FILE *fp = fopen(filename, "w");
    if (fp == NULL) {
        printf("Error opening file %s\n", filename);
        return;
    }
    
    // Header
    fprintf(fp, "time,x,y,psi,surge,yaw,t_param,tau_port,tau_stbd,dt,slack_u\n");
    
    // Data
    for (int i = 0; i < nsim; i++) {
        fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                i * DT,
                simX[i][0], simX[i][1], simX[i][2], simX[i][3], simX[i][4], simX[i][5],
                simU[i][0], simU[i][1], simU[i][2], simU[i][3]);
    }
    
    // Final state
    fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,0,0,0,0\n",
            nsim * DT,
            simX[nsim][0], simX[nsim][1], simX[nsim][2], 
            simX[nsim][3], simX[nsim][4], simX[nsim][5]);
    
    fclose(fp);
    printf("Trajectory saved to %s\n", filename);
}

int main() {
    int status = 0;
    
    printf("=== ASV Spline Tracking - Closed-Loop Simulation ===\n\n");
    
    // === DEFINE SPLINE ===
    double p0[2] = {0.0, 0.0};
    double p1[2] = {2.0, 1.0};
    double p2[2] = {6.0, -3.0};
    double p3[2] = {10.0, 2.0};
    
    double spline_params[NP];
    get_catmull_rom_segment(p0, p1, p2, p3, spline_params);
    
    printf("Spline parameters:\n");
    for (int i = 0; i < NP; i++) {
        printf("  p[%d] = %.6f\n", i, spline_params[i]);
    }
    printf("\n");
    
    // === CREATE OCP SOLVER ===
    printf("Creating OCP solver...\n");
    asv_spline_tracking_solver_capsule *ocp_capsule = asv_spline_tracking_acados_create_capsule();
    status = asv_spline_tracking_acados_create_with_discretization(ocp_capsule, N_HORIZON, NULL);
    
    if (status) {
        printf("OCP solver creation failed with status %d\n", status);
        return 1;
    }
    
    ocp_nlp_config *nlp_config = asv_spline_tracking_acados_get_nlp_config(ocp_capsule);
    ocp_nlp_dims *nlp_dims = asv_spline_tracking_acados_get_nlp_dims(ocp_capsule);
    ocp_nlp_in *nlp_in = asv_spline_tracking_acados_get_nlp_in(ocp_capsule);
    ocp_nlp_out *nlp_out = asv_spline_tracking_acados_get_nlp_out(ocp_capsule);
    ocp_nlp_solver *nlp_solver = asv_spline_tracking_acados_get_nlp_solver(ocp_capsule);
    
    // === CREATE SIMULATOR ===
    printf("Creating simulator...\n");
    asv_spline_tracking_sim_solver_capsule *sim_capsule = asv_spline_tracking_acados_sim_solver_create_capsule();
    status = asv_spline_tracking_acados_sim_create(sim_capsule);
    
    if (status) {
        printf("Simulator creation failed with status %d\n", status);
        return 1;
    }
    
    sim_config *sim_config = asv_spline_tracking_acados_get_sim_config(sim_capsule);
    sim_in *sim_in = asv_spline_tracking_acados_get_sim_in(sim_capsule);
    sim_out *sim_out = asv_spline_tracking_acados_get_sim_out(sim_capsule);
    void *sim_dims = asv_spline_tracking_acados_get_sim_dims(sim_capsule);
    
    // === INITIAL CONDITIONS ===
    double x0[NX] = {5.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    
    printf("Initial state: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n\n",
           x0[0], x0[1], x0[2], x0[3], x0[4], x0[5]);
    
    // === SIMULATION SETUP ===
    int Nsim = (int)(T_SIM / DT);
    printf("Simulation parameters:\n");
    printf("  Total time: %.1f s\n", T_SIM);
    printf("  Time step: %.4f s\n", DT);
    printf("  Number of steps: %d\n", Nsim);
    printf("  MPC horizon: %d nodes\n\n", N_HORIZON);
    
    // Allocate memory for trajectories
    double **simX = (double**)malloc((Nsim + 1) * sizeof(double*));
    double **simU = (double**)malloc(Nsim * sizeof(double*));
    double *t_prep = (double*)malloc(Nsim * sizeof(double));
    double *t_feedback = (double*)malloc(Nsim * sizeof(double));
    
    for (int i = 0; i <= Nsim; i++) {
        simX[i] = (double*)malloc(NX * sizeof(double));
    }
    for (int i = 0; i < Nsim; i++) {
        simU[i] = (double*)malloc(NU * sizeof(double));
    }
    
    // Set initial state
    memcpy(simX[0], x0, NX * sizeof(double));
    
    // === SET SPLINE PARAMETERS FOR ALL STAGES ===
    for (int i = 0; i <= N_HORIZON; i++) {
        asv_spline_tracking_acados_update_params(ocp_capsule, i, spline_params, NP);
    }
    asv_spline_tracking_acados_sim_update_params(sim_capsule, spline_params, NP);
    
    // === CLOSED-LOOP SIMULATION ===
    printf("Running closed-loop simulation...\n\n");
    
    clock_t start_total = clock();
    
    for (int i = 0; i < Nsim; i++) {
        // Set initial state constraint
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", simX[i]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", simX[i]);
        
        // === RTI PHASE 1: PREPARATION ===
        int rti_phase = 1;
        clock_t prep_start = clock();
        ocp_nlp_solver_opts_set(nlp_config, ocp_capsule->nlp_opts, "rti_phase", &rti_phase);
        status = asv_spline_tracking_acados_solve(ocp_capsule);
        clock_t prep_end = clock();
        t_prep[i] = ((double)(prep_end - prep_start)) / CLOCKS_PER_SEC;
        
        if (status != 0 && status != 2 && status != 5) {
            printf("Warning: Preparation phase returned status %d at step %d\n", status, i);
        }
        
        // === RTI PHASE 2: FEEDBACK ===
        rti_phase = 2;
        clock_t fb_start = clock();
        ocp_nlp_solver_opts_set(nlp_config, ocp_capsule->nlp_opts, "rti_phase", &rti_phase);
        status = asv_spline_tracking_acados_solve(ocp_capsule);
        clock_t fb_end = clock();
        t_feedback[i] = ((double)(fb_end - fb_start)) / CLOCKS_PER_SEC;
        
        if (status != 0 && status != 2 && status != 5) {
            printf("Warning: Feedback phase returned status %d at step %d\n", status, i);
        }
                
        // Get optimal control
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", simU[i]);
        
        // Simulate system forward
        sim_in_set(sim_config, sim_dims, sim_in, "x", simX[i]);
        sim_in_set(sim_config, sim_dims, sim_in, "u", simU[i]);
        
        status = asv_spline_tracking_acados_sim_solve(sim_capsule);
        if (status != 0) {
            printf("Simulation failed with status %d at step %d\n", status, i);
            break;
        }
        
        sim_out_get(sim_config, sim_dims, sim_out, "x", simX[i + 1]);
        
        // Print progress
        if ((i + 1) % 50 == 0 || i == 0) {
            printf("Step %4d/%d: t=%.3f, pos=(%.2f, %.2f), prep=%.2fms, feedback=%.2fms\n",
                   i + 1, Nsim, simX[i][5], simX[i][0], simX[i][1],
                   t_prep[i] * 1000.0, t_feedback[i] * 1000.0);
        }
    }
    
    clock_t end_total = clock();
    double total_time = ((double)(end_total - start_total)) / CLOCKS_PER_SEC;
    
    // === COMPUTE STATISTICS ===
    printf("\n=== Simulation Complete ===\n\n");
    
    // Timing statistics
    double min_prep = t_prep[0], max_prep = t_prep[0], sum_prep = 0.0;
    double min_fb = t_feedback[0], max_fb = t_feedback[0], sum_fb = 0.0;
    
    for (int i = 0; i < Nsim; i++) {
        if (t_prep[i] < min_prep) min_prep = t_prep[i];
        if (t_prep[i] > max_prep) max_prep = t_prep[i];
        sum_prep += t_prep[i];
        
        if (t_feedback[i] < min_fb) min_fb = t_feedback[i];
        if (t_feedback[i] > max_fb) max_fb = t_feedback[i];
        sum_fb += t_feedback[i];
    }
    
    printf("Timing Statistics:\n");
    printf("  Preparation phase [ms]:\n");
    printf("    min: %.3f, avg: %.3f, max: %.3f\n", 
           min_prep * 1000, (sum_prep / Nsim) * 1000, max_prep * 1000);
    printf("  Feedback phase [ms]:\n");
    printf("    min: %.3f, avg: %.3f, max: %.3f\n",
           min_fb * 1000, (sum_fb / Nsim) * 1000, max_fb * 1000);
    printf("  Total per iteration [ms]:\n");
    printf("    min: %.3f, avg: %.3f, max: %.3f\n",
           (min_prep + min_fb) * 1000, 
           ((sum_prep + sum_fb) / Nsim) * 1000,
           (max_prep + max_fb) * 1000);
    printf("  Wall-clock time: %.2f s\n\n", total_time);
    
    // Final state
    printf("Final Results:\n");
    printf("  t parameter: %.3f\n", simX[Nsim][5]);
    printf("  Position: (%.2f, %.2f)\n", simX[Nsim][0], simX[Nsim][1]);
    
    // Compute final tracking error
    double final_spline_pos[2];
    evaluate_spline(simX[Nsim][5], spline_params, final_spline_pos);
    double dx = simX[Nsim][0] - final_spline_pos[0];
    double dy = simX[Nsim][1] - final_spline_pos[1];
    double crosstrack_error = sqrt(dx*dx + dy*dy);
    printf("  Crosstrack error: %.3f m\n\n", crosstrack_error);
    
    // Save results
    save_trajectory_csv("asv_trajectory.csv", simX, simU, Nsim);
    
    // === CLEANUP ===
    asv_spline_tracking_acados_free(ocp_capsule);
    asv_spline_tracking_acados_free_capsule(ocp_capsule);
    asv_spline_tracking_acados_sim_free(sim_capsule);
    asv_spline_tracking_acados_sim_solver_free_capsule(sim_capsule);
    
    for (int i = 0; i <= Nsim; i++) free(simX[i]);
    for (int i = 0; i < Nsim; i++) free(simU[i]);
    free(simX);
    free(simU);
    free(t_prep);
    free(t_feedback);
    
    printf("Done!\n");
    
    return 0;
}