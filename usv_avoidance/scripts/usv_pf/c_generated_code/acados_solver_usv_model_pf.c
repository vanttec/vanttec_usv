/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "usv_model_pf_model/usv_model_pf_model.h"





#include "acados_solver_usv_model_pf.h"

#define NX     14
#define NZ     0
#define NU     2
#define NP     0
#define NBX    5
#define NBX0   14
#define NBU    2
#define NSBX   0
#define NSBU   0
#define NSH    0
#define NSG    0
#define NSPHI  0
#define NSHN   0
#define NSGN   0
#define NSPHIN 0
#define NSBXN  0
#define NS     0
#define NSN    0
#define NG     0
#define NBXN   0
#define NGN    0
#define NY     16
#define NYN    14
#define N      100
#define NH     0
#define NPHI   0
#define NHN    0
#define NPHIN  0
#define NR     0


// ** global data **
ocp_nlp_in * nlp_in;
ocp_nlp_out * nlp_out;
ocp_nlp_solver * nlp_solver;
void * nlp_opts;
ocp_nlp_plan * nlp_solver_plan;
ocp_nlp_config * nlp_config;
ocp_nlp_dims * nlp_dims;

// number of expected runtime parameters
const unsigned int nlp_np = NP;


external_function_param_casadi * forw_vde_casadi;
external_function_param_casadi * expl_ode_fun;



external_function_param_casadi * nl_constr_h_fun;
external_function_param_casadi * nl_constr_h_fun_jac;


external_function_param_casadi nl_constr_h_e_fun_jac;
external_function_param_casadi nl_constr_h_e_fun;


int acados_create()
{
    int status = 0;

    /************************************************
    *  plan & config
    ************************************************/
    nlp_solver_plan = ocp_nlp_plan_create(N);
    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    for (int i = 0; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
    nlp_config = ocp_nlp_config_create(*nlp_solver_plan);


    /************************************************
    *  dimensions
    ************************************************/
    int nx[N+1];
    int nu[N+1];
    int nbx[N+1];
    int nbu[N+1];
    int nsbx[N+1];
    int nsbu[N+1];
    int nsg[N+1];
    int nsh[N+1];
    int nsphi[N+1];
    int ns[N+1];
    int ng[N+1];
    int nh[N+1];
    int nphi[N+1];
    int nz[N+1];
    int ny[N+1];
    int nr[N+1];
    int nbxe[N+1];

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i] = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 14;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);



    /************************************************
    *  external functions
    ************************************************/


    // explicit ode
    forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        forw_vde_casadi[i].casadi_fun = &usv_model_pf_expl_vde_forw;
        forw_vde_casadi[i].casadi_n_in = &usv_model_pf_expl_vde_forw_n_in;
        forw_vde_casadi[i].casadi_n_out = &usv_model_pf_expl_vde_forw_n_out;
        forw_vde_casadi[i].casadi_sparsity_in = &usv_model_pf_expl_vde_forw_sparsity_in;
        forw_vde_casadi[i].casadi_sparsity_out = &usv_model_pf_expl_vde_forw_sparsity_out;
        forw_vde_casadi[i].casadi_work = &usv_model_pf_expl_vde_forw_work;
        external_function_param_casadi_create(&forw_vde_casadi[i], 0);
    }

    expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        expl_ode_fun[i].casadi_fun = &usv_model_pf_expl_ode_fun;
        expl_ode_fun[i].casadi_n_in = &usv_model_pf_expl_ode_fun_n_in;
        expl_ode_fun[i].casadi_n_out = &usv_model_pf_expl_ode_fun_n_out;
        expl_ode_fun[i].casadi_sparsity_in = &usv_model_pf_expl_ode_fun_sparsity_in;
        expl_ode_fun[i].casadi_sparsity_out = &usv_model_pf_expl_ode_fun_sparsity_out;
        expl_ode_fun[i].casadi_work = &usv_model_pf_expl_ode_fun_work;
        external_function_param_casadi_create(&expl_ode_fun[i], 0);
    }



    /************************************************
    *  nlp_in
    ************************************************/
    nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);

    double time_steps[N];
    time_steps[0] = 0.01;
    time_steps[1] = 0.01;
    time_steps[2] = 0.01;
    time_steps[3] = 0.01;
    time_steps[4] = 0.01;
    time_steps[5] = 0.01;
    time_steps[6] = 0.01;
    time_steps[7] = 0.01;
    time_steps[8] = 0.01;
    time_steps[9] = 0.01;
    time_steps[10] = 0.01;
    time_steps[11] = 0.01;
    time_steps[12] = 0.01;
    time_steps[13] = 0.01;
    time_steps[14] = 0.01;
    time_steps[15] = 0.01;
    time_steps[16] = 0.01;
    time_steps[17] = 0.01;
    time_steps[18] = 0.01;
    time_steps[19] = 0.01;
    time_steps[20] = 0.01;
    time_steps[21] = 0.01;
    time_steps[22] = 0.01;
    time_steps[23] = 0.01;
    time_steps[24] = 0.01;
    time_steps[25] = 0.01;
    time_steps[26] = 0.01;
    time_steps[27] = 0.01;
    time_steps[28] = 0.01;
    time_steps[29] = 0.01;
    time_steps[30] = 0.01;
    time_steps[31] = 0.01;
    time_steps[32] = 0.01;
    time_steps[33] = 0.01;
    time_steps[34] = 0.01;
    time_steps[35] = 0.01;
    time_steps[36] = 0.01;
    time_steps[37] = 0.01;
    time_steps[38] = 0.01;
    time_steps[39] = 0.01;
    time_steps[40] = 0.01;
    time_steps[41] = 0.01;
    time_steps[42] = 0.01;
    time_steps[43] = 0.01;
    time_steps[44] = 0.01;
    time_steps[45] = 0.01;
    time_steps[46] = 0.01;
    time_steps[47] = 0.01;
    time_steps[48] = 0.01;
    time_steps[49] = 0.01;
    time_steps[50] = 0.01;
    time_steps[51] = 0.01;
    time_steps[52] = 0.01;
    time_steps[53] = 0.01;
    time_steps[54] = 0.01;
    time_steps[55] = 0.01;
    time_steps[56] = 0.01;
    time_steps[57] = 0.01;
    time_steps[58] = 0.01;
    time_steps[59] = 0.01;
    time_steps[60] = 0.01;
    time_steps[61] = 0.01;
    time_steps[62] = 0.01;
    time_steps[63] = 0.01;
    time_steps[64] = 0.01;
    time_steps[65] = 0.01;
    time_steps[66] = 0.01;
    time_steps[67] = 0.01;
    time_steps[68] = 0.01;
    time_steps[69] = 0.01;
    time_steps[70] = 0.01;
    time_steps[71] = 0.01;
    time_steps[72] = 0.01;
    time_steps[73] = 0.01;
    time_steps[74] = 0.01;
    time_steps[75] = 0.01;
    time_steps[76] = 0.01;
    time_steps[77] = 0.01;
    time_steps[78] = 0.01;
    time_steps[79] = 0.01;
    time_steps[80] = 0.01;
    time_steps[81] = 0.01;
    time_steps[82] = 0.01;
    time_steps[83] = 0.01;
    time_steps[84] = 0.01;
    time_steps[85] = 0.01;
    time_steps[86] = 0.01;
    time_steps[87] = 0.01;
    time_steps[88] = 0.01;
    time_steps[89] = 0.01;
    time_steps[90] = 0.01;
    time_steps[91] = 0.01;
    time_steps[92] = 0.01;
    time_steps[93] = 0.01;
    time_steps[94] = 0.01;
    time_steps[95] = 0.01;
    time_steps[96] = 0.01;
    time_steps[97] = 0.01;
    time_steps[98] = 0.01;
    time_steps[99] = 0.01;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_steps[i]);
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &expl_ode_fun[i]);
    
    }


    /**** Cost ****/

    double W[NY*NY];
    
    W[0+(NY) * 0] = 0;
    W[0+(NY) * 1] = 0;
    W[0+(NY) * 2] = 0;
    W[0+(NY) * 3] = 0;
    W[0+(NY) * 4] = 0;
    W[0+(NY) * 5] = 0;
    W[0+(NY) * 6] = 0;
    W[0+(NY) * 7] = 0;
    W[0+(NY) * 8] = 0;
    W[0+(NY) * 9] = 0;
    W[0+(NY) * 10] = 0;
    W[0+(NY) * 11] = 0;
    W[0+(NY) * 12] = 0;
    W[0+(NY) * 13] = 0;
    W[0+(NY) * 14] = 0;
    W[0+(NY) * 15] = 0;
    W[1+(NY) * 0] = 0;
    W[1+(NY) * 1] = 0.3;
    W[1+(NY) * 2] = 0;
    W[1+(NY) * 3] = 0;
    W[1+(NY) * 4] = 0;
    W[1+(NY) * 5] = 0;
    W[1+(NY) * 6] = 0;
    W[1+(NY) * 7] = 0;
    W[1+(NY) * 8] = 0;
    W[1+(NY) * 9] = 0;
    W[1+(NY) * 10] = 0;
    W[1+(NY) * 11] = 0;
    W[1+(NY) * 12] = 0;
    W[1+(NY) * 13] = 0;
    W[1+(NY) * 14] = 0;
    W[1+(NY) * 15] = 0;
    W[2+(NY) * 0] = 0;
    W[2+(NY) * 1] = 0;
    W[2+(NY) * 2] = 0.3;
    W[2+(NY) * 3] = 0;
    W[2+(NY) * 4] = 0;
    W[2+(NY) * 5] = 0;
    W[2+(NY) * 6] = 0;
    W[2+(NY) * 7] = 0;
    W[2+(NY) * 8] = 0;
    W[2+(NY) * 9] = 0;
    W[2+(NY) * 10] = 0;
    W[2+(NY) * 11] = 0;
    W[2+(NY) * 12] = 0;
    W[2+(NY) * 13] = 0;
    W[2+(NY) * 14] = 0;
    W[2+(NY) * 15] = 0;
    W[3+(NY) * 0] = 0;
    W[3+(NY) * 1] = 0;
    W[3+(NY) * 2] = 0;
    W[3+(NY) * 3] = 80;
    W[3+(NY) * 4] = 0;
    W[3+(NY) * 5] = 0;
    W[3+(NY) * 6] = 0;
    W[3+(NY) * 7] = 0;
    W[3+(NY) * 8] = 0;
    W[3+(NY) * 9] = 0;
    W[3+(NY) * 10] = 0;
    W[3+(NY) * 11] = 0;
    W[3+(NY) * 12] = 0;
    W[3+(NY) * 13] = 0;
    W[3+(NY) * 14] = 0;
    W[3+(NY) * 15] = 0;
    W[4+(NY) * 0] = 0;
    W[4+(NY) * 1] = 0;
    W[4+(NY) * 2] = 0;
    W[4+(NY) * 3] = 0;
    W[4+(NY) * 4] = 0;
    W[4+(NY) * 5] = 0;
    W[4+(NY) * 6] = 0;
    W[4+(NY) * 7] = 0;
    W[4+(NY) * 8] = 0;
    W[4+(NY) * 9] = 0;
    W[4+(NY) * 10] = 0;
    W[4+(NY) * 11] = 0;
    W[4+(NY) * 12] = 0;
    W[4+(NY) * 13] = 0;
    W[4+(NY) * 14] = 0;
    W[4+(NY) * 15] = 0;
    W[5+(NY) * 0] = 0;
    W[5+(NY) * 1] = 0;
    W[5+(NY) * 2] = 0;
    W[5+(NY) * 3] = 0;
    W[5+(NY) * 4] = 0;
    W[5+(NY) * 5] = 0;
    W[5+(NY) * 6] = 0;
    W[5+(NY) * 7] = 0;
    W[5+(NY) * 8] = 0;
    W[5+(NY) * 9] = 0;
    W[5+(NY) * 10] = 0;
    W[5+(NY) * 11] = 0;
    W[5+(NY) * 12] = 0;
    W[5+(NY) * 13] = 0;
    W[5+(NY) * 14] = 0;
    W[5+(NY) * 15] = 0;
    W[6+(NY) * 0] = 0;
    W[6+(NY) * 1] = 0;
    W[6+(NY) * 2] = 0;
    W[6+(NY) * 3] = 0;
    W[6+(NY) * 4] = 0;
    W[6+(NY) * 5] = 0;
    W[6+(NY) * 6] = 0.8;
    W[6+(NY) * 7] = 0;
    W[6+(NY) * 8] = 0;
    W[6+(NY) * 9] = 0;
    W[6+(NY) * 10] = 0;
    W[6+(NY) * 11] = 0;
    W[6+(NY) * 12] = 0;
    W[6+(NY) * 13] = 0;
    W[6+(NY) * 14] = 0;
    W[6+(NY) * 15] = 0;
    W[7+(NY) * 0] = 0;
    W[7+(NY) * 1] = 0;
    W[7+(NY) * 2] = 0;
    W[7+(NY) * 3] = 0;
    W[7+(NY) * 4] = 0;
    W[7+(NY) * 5] = 0;
    W[7+(NY) * 6] = 0;
    W[7+(NY) * 7] = 0;
    W[7+(NY) * 8] = 0;
    W[7+(NY) * 9] = 0;
    W[7+(NY) * 10] = 0;
    W[7+(NY) * 11] = 0;
    W[7+(NY) * 12] = 0;
    W[7+(NY) * 13] = 0;
    W[7+(NY) * 14] = 0;
    W[7+(NY) * 15] = 0;
    W[8+(NY) * 0] = 0;
    W[8+(NY) * 1] = 0;
    W[8+(NY) * 2] = 0;
    W[8+(NY) * 3] = 0;
    W[8+(NY) * 4] = 0;
    W[8+(NY) * 5] = 0;
    W[8+(NY) * 6] = 0;
    W[8+(NY) * 7] = 0;
    W[8+(NY) * 8] = 0;
    W[8+(NY) * 9] = 0;
    W[8+(NY) * 10] = 0;
    W[8+(NY) * 11] = 0;
    W[8+(NY) * 12] = 0;
    W[8+(NY) * 13] = 0;
    W[8+(NY) * 14] = 0;
    W[8+(NY) * 15] = 0;
    W[9+(NY) * 0] = 0;
    W[9+(NY) * 1] = 0;
    W[9+(NY) * 2] = 0;
    W[9+(NY) * 3] = 0;
    W[9+(NY) * 4] = 0;
    W[9+(NY) * 5] = 0;
    W[9+(NY) * 6] = 0;
    W[9+(NY) * 7] = 0;
    W[9+(NY) * 8] = 0;
    W[9+(NY) * 9] = 0;
    W[9+(NY) * 10] = 0;
    W[9+(NY) * 11] = 0;
    W[9+(NY) * 12] = 0;
    W[9+(NY) * 13] = 0;
    W[9+(NY) * 14] = 0;
    W[9+(NY) * 15] = 0;
    W[10+(NY) * 0] = 0;
    W[10+(NY) * 1] = 0;
    W[10+(NY) * 2] = 0;
    W[10+(NY) * 3] = 0;
    W[10+(NY) * 4] = 0;
    W[10+(NY) * 5] = 0;
    W[10+(NY) * 6] = 0;
    W[10+(NY) * 7] = 0;
    W[10+(NY) * 8] = 0;
    W[10+(NY) * 9] = 0;
    W[10+(NY) * 10] = 0;
    W[10+(NY) * 11] = 0;
    W[10+(NY) * 12] = 0;
    W[10+(NY) * 13] = 0;
    W[10+(NY) * 14] = 0;
    W[10+(NY) * 15] = 0;
    W[11+(NY) * 0] = 0;
    W[11+(NY) * 1] = 0;
    W[11+(NY) * 2] = 0;
    W[11+(NY) * 3] = 0;
    W[11+(NY) * 4] = 0;
    W[11+(NY) * 5] = 0;
    W[11+(NY) * 6] = 0;
    W[11+(NY) * 7] = 0;
    W[11+(NY) * 8] = 0;
    W[11+(NY) * 9] = 0;
    W[11+(NY) * 10] = 0;
    W[11+(NY) * 11] = 0;
    W[11+(NY) * 12] = 0;
    W[11+(NY) * 13] = 0;
    W[11+(NY) * 14] = 0;
    W[11+(NY) * 15] = 0;
    W[12+(NY) * 0] = 0;
    W[12+(NY) * 1] = 0;
    W[12+(NY) * 2] = 0;
    W[12+(NY) * 3] = 0;
    W[12+(NY) * 4] = 0;
    W[12+(NY) * 5] = 0;
    W[12+(NY) * 6] = 0;
    W[12+(NY) * 7] = 0;
    W[12+(NY) * 8] = 0;
    W[12+(NY) * 9] = 0;
    W[12+(NY) * 10] = 0;
    W[12+(NY) * 11] = 0;
    W[12+(NY) * 12] = 0.0001;
    W[12+(NY) * 13] = 0;
    W[12+(NY) * 14] = 0;
    W[12+(NY) * 15] = 0;
    W[13+(NY) * 0] = 0;
    W[13+(NY) * 1] = 0;
    W[13+(NY) * 2] = 0;
    W[13+(NY) * 3] = 0;
    W[13+(NY) * 4] = 0;
    W[13+(NY) * 5] = 0;
    W[13+(NY) * 6] = 0;
    W[13+(NY) * 7] = 0;
    W[13+(NY) * 8] = 0;
    W[13+(NY) * 9] = 0;
    W[13+(NY) * 10] = 0;
    W[13+(NY) * 11] = 0;
    W[13+(NY) * 12] = 0;
    W[13+(NY) * 13] = 0.0001;
    W[13+(NY) * 14] = 0;
    W[13+(NY) * 15] = 0;
    W[14+(NY) * 0] = 0;
    W[14+(NY) * 1] = 0;
    W[14+(NY) * 2] = 0;
    W[14+(NY) * 3] = 0;
    W[14+(NY) * 4] = 0;
    W[14+(NY) * 5] = 0;
    W[14+(NY) * 6] = 0;
    W[14+(NY) * 7] = 0;
    W[14+(NY) * 8] = 0;
    W[14+(NY) * 9] = 0;
    W[14+(NY) * 10] = 0;
    W[14+(NY) * 11] = 0;
    W[14+(NY) * 12] = 0;
    W[14+(NY) * 13] = 0;
    W[14+(NY) * 14] = 0;
    W[14+(NY) * 15] = 0;
    W[15+(NY) * 0] = 0;
    W[15+(NY) * 1] = 0;
    W[15+(NY) * 2] = 0;
    W[15+(NY) * 3] = 0;
    W[15+(NY) * 4] = 0;
    W[15+(NY) * 5] = 0;
    W[15+(NY) * 6] = 0;
    W[15+(NY) * 7] = 0;
    W[15+(NY) * 8] = 0;
    W[15+(NY) * 9] = 0;
    W[15+(NY) * 10] = 0;
    W[15+(NY) * 11] = 0;
    W[15+(NY) * 12] = 0;
    W[15+(NY) * 13] = 0;
    W[15+(NY) * 14] = 0;
    W[15+(NY) * 15] = 0;

    double yref[NY];
    
    yref[0] = 0;
    yref[1] = 0;
    yref[2] = 0;
    yref[3] = 0;
    yref[4] = 0;
    yref[5] = 0;
    yref[6] = 0;
    yref[7] = 0;
    yref[8] = 0;
    yref[9] = 0;
    yref[10] = 0;
    yref[11] = 0;
    yref[12] = 0;
    yref[13] = 0;
    yref[14] = 0;
    yref[15] = 0;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }


    double Vx[NY*NX];
    
    Vx[0+(NY) * 0] = 1;
    Vx[0+(NY) * 1] = 0;
    Vx[0+(NY) * 2] = 0;
    Vx[0+(NY) * 3] = 0;
    Vx[0+(NY) * 4] = 0;
    Vx[0+(NY) * 5] = 0;
    Vx[0+(NY) * 6] = 0;
    Vx[0+(NY) * 7] = 0;
    Vx[0+(NY) * 8] = 0;
    Vx[0+(NY) * 9] = 0;
    Vx[0+(NY) * 10] = 0;
    Vx[0+(NY) * 11] = 0;
    Vx[0+(NY) * 12] = 0;
    Vx[0+(NY) * 13] = 0;
    Vx[1+(NY) * 0] = 0;
    Vx[1+(NY) * 1] = 1;
    Vx[1+(NY) * 2] = 0;
    Vx[1+(NY) * 3] = 0;
    Vx[1+(NY) * 4] = 0;
    Vx[1+(NY) * 5] = 0;
    Vx[1+(NY) * 6] = 0;
    Vx[1+(NY) * 7] = 0;
    Vx[1+(NY) * 8] = 0;
    Vx[1+(NY) * 9] = 0;
    Vx[1+(NY) * 10] = 0;
    Vx[1+(NY) * 11] = 0;
    Vx[1+(NY) * 12] = 0;
    Vx[1+(NY) * 13] = 0;
    Vx[2+(NY) * 0] = 0;
    Vx[2+(NY) * 1] = 0;
    Vx[2+(NY) * 2] = 1;
    Vx[2+(NY) * 3] = 0;
    Vx[2+(NY) * 4] = 0;
    Vx[2+(NY) * 5] = 0;
    Vx[2+(NY) * 6] = 0;
    Vx[2+(NY) * 7] = 0;
    Vx[2+(NY) * 8] = 0;
    Vx[2+(NY) * 9] = 0;
    Vx[2+(NY) * 10] = 0;
    Vx[2+(NY) * 11] = 0;
    Vx[2+(NY) * 12] = 0;
    Vx[2+(NY) * 13] = 0;
    Vx[3+(NY) * 0] = 0;
    Vx[3+(NY) * 1] = 0;
    Vx[3+(NY) * 2] = 0;
    Vx[3+(NY) * 3] = 1;
    Vx[3+(NY) * 4] = 0;
    Vx[3+(NY) * 5] = 0;
    Vx[3+(NY) * 6] = 0;
    Vx[3+(NY) * 7] = 0;
    Vx[3+(NY) * 8] = 0;
    Vx[3+(NY) * 9] = 0;
    Vx[3+(NY) * 10] = 0;
    Vx[3+(NY) * 11] = 0;
    Vx[3+(NY) * 12] = 0;
    Vx[3+(NY) * 13] = 0;
    Vx[4+(NY) * 0] = 0;
    Vx[4+(NY) * 1] = 0;
    Vx[4+(NY) * 2] = 0;
    Vx[4+(NY) * 3] = 0;
    Vx[4+(NY) * 4] = 1;
    Vx[4+(NY) * 5] = 0;
    Vx[4+(NY) * 6] = 0;
    Vx[4+(NY) * 7] = 0;
    Vx[4+(NY) * 8] = 0;
    Vx[4+(NY) * 9] = 0;
    Vx[4+(NY) * 10] = 0;
    Vx[4+(NY) * 11] = 0;
    Vx[4+(NY) * 12] = 0;
    Vx[4+(NY) * 13] = 0;
    Vx[5+(NY) * 0] = 0;
    Vx[5+(NY) * 1] = 0;
    Vx[5+(NY) * 2] = 0;
    Vx[5+(NY) * 3] = 0;
    Vx[5+(NY) * 4] = 0;
    Vx[5+(NY) * 5] = 1;
    Vx[5+(NY) * 6] = 0;
    Vx[5+(NY) * 7] = 0;
    Vx[5+(NY) * 8] = 0;
    Vx[5+(NY) * 9] = 0;
    Vx[5+(NY) * 10] = 0;
    Vx[5+(NY) * 11] = 0;
    Vx[5+(NY) * 12] = 0;
    Vx[5+(NY) * 13] = 0;
    Vx[6+(NY) * 0] = 0;
    Vx[6+(NY) * 1] = 0;
    Vx[6+(NY) * 2] = 0;
    Vx[6+(NY) * 3] = 0;
    Vx[6+(NY) * 4] = 0;
    Vx[6+(NY) * 5] = 0;
    Vx[6+(NY) * 6] = 1;
    Vx[6+(NY) * 7] = 0;
    Vx[6+(NY) * 8] = 0;
    Vx[6+(NY) * 9] = 0;
    Vx[6+(NY) * 10] = 0;
    Vx[6+(NY) * 11] = 0;
    Vx[6+(NY) * 12] = 0;
    Vx[6+(NY) * 13] = 0;
    Vx[7+(NY) * 0] = 0;
    Vx[7+(NY) * 1] = 0;
    Vx[7+(NY) * 2] = 0;
    Vx[7+(NY) * 3] = 0;
    Vx[7+(NY) * 4] = 0;
    Vx[7+(NY) * 5] = 0;
    Vx[7+(NY) * 6] = 0;
    Vx[7+(NY) * 7] = 1;
    Vx[7+(NY) * 8] = 0;
    Vx[7+(NY) * 9] = 0;
    Vx[7+(NY) * 10] = 0;
    Vx[7+(NY) * 11] = 0;
    Vx[7+(NY) * 12] = 0;
    Vx[7+(NY) * 13] = 0;
    Vx[8+(NY) * 0] = 0;
    Vx[8+(NY) * 1] = 0;
    Vx[8+(NY) * 2] = 0;
    Vx[8+(NY) * 3] = 0;
    Vx[8+(NY) * 4] = 0;
    Vx[8+(NY) * 5] = 0;
    Vx[8+(NY) * 6] = 0;
    Vx[8+(NY) * 7] = 0;
    Vx[8+(NY) * 8] = 1;
    Vx[8+(NY) * 9] = 0;
    Vx[8+(NY) * 10] = 0;
    Vx[8+(NY) * 11] = 0;
    Vx[8+(NY) * 12] = 0;
    Vx[8+(NY) * 13] = 0;
    Vx[9+(NY) * 0] = 0;
    Vx[9+(NY) * 1] = 0;
    Vx[9+(NY) * 2] = 0;
    Vx[9+(NY) * 3] = 0;
    Vx[9+(NY) * 4] = 0;
    Vx[9+(NY) * 5] = 0;
    Vx[9+(NY) * 6] = 0;
    Vx[9+(NY) * 7] = 0;
    Vx[9+(NY) * 8] = 0;
    Vx[9+(NY) * 9] = 1;
    Vx[9+(NY) * 10] = 0;
    Vx[9+(NY) * 11] = 0;
    Vx[9+(NY) * 12] = 0;
    Vx[9+(NY) * 13] = 0;
    Vx[10+(NY) * 0] = 0;
    Vx[10+(NY) * 1] = 0;
    Vx[10+(NY) * 2] = 0;
    Vx[10+(NY) * 3] = 0;
    Vx[10+(NY) * 4] = 0;
    Vx[10+(NY) * 5] = 0;
    Vx[10+(NY) * 6] = 0;
    Vx[10+(NY) * 7] = 0;
    Vx[10+(NY) * 8] = 0;
    Vx[10+(NY) * 9] = 0;
    Vx[10+(NY) * 10] = 1;
    Vx[10+(NY) * 11] = 0;
    Vx[10+(NY) * 12] = 0;
    Vx[10+(NY) * 13] = 0;
    Vx[11+(NY) * 0] = 0;
    Vx[11+(NY) * 1] = 0;
    Vx[11+(NY) * 2] = 0;
    Vx[11+(NY) * 3] = 0;
    Vx[11+(NY) * 4] = 0;
    Vx[11+(NY) * 5] = 0;
    Vx[11+(NY) * 6] = 0;
    Vx[11+(NY) * 7] = 0;
    Vx[11+(NY) * 8] = 0;
    Vx[11+(NY) * 9] = 0;
    Vx[11+(NY) * 10] = 0;
    Vx[11+(NY) * 11] = 1;
    Vx[11+(NY) * 12] = 0;
    Vx[11+(NY) * 13] = 0;
    Vx[12+(NY) * 0] = 0;
    Vx[12+(NY) * 1] = 0;
    Vx[12+(NY) * 2] = 0;
    Vx[12+(NY) * 3] = 0;
    Vx[12+(NY) * 4] = 0;
    Vx[12+(NY) * 5] = 0;
    Vx[12+(NY) * 6] = 0;
    Vx[12+(NY) * 7] = 0;
    Vx[12+(NY) * 8] = 0;
    Vx[12+(NY) * 9] = 0;
    Vx[12+(NY) * 10] = 0;
    Vx[12+(NY) * 11] = 0;
    Vx[12+(NY) * 12] = 1;
    Vx[12+(NY) * 13] = 0;
    Vx[13+(NY) * 0] = 0;
    Vx[13+(NY) * 1] = 0;
    Vx[13+(NY) * 2] = 0;
    Vx[13+(NY) * 3] = 0;
    Vx[13+(NY) * 4] = 0;
    Vx[13+(NY) * 5] = 0;
    Vx[13+(NY) * 6] = 0;
    Vx[13+(NY) * 7] = 0;
    Vx[13+(NY) * 8] = 0;
    Vx[13+(NY) * 9] = 0;
    Vx[13+(NY) * 10] = 0;
    Vx[13+(NY) * 11] = 0;
    Vx[13+(NY) * 12] = 0;
    Vx[13+(NY) * 13] = 1;
    Vx[14+(NY) * 0] = 0;
    Vx[14+(NY) * 1] = 0;
    Vx[14+(NY) * 2] = 0;
    Vx[14+(NY) * 3] = 0;
    Vx[14+(NY) * 4] = 0;
    Vx[14+(NY) * 5] = 0;
    Vx[14+(NY) * 6] = 0;
    Vx[14+(NY) * 7] = 0;
    Vx[14+(NY) * 8] = 0;
    Vx[14+(NY) * 9] = 0;
    Vx[14+(NY) * 10] = 0;
    Vx[14+(NY) * 11] = 0;
    Vx[14+(NY) * 12] = 0;
    Vx[14+(NY) * 13] = 0;
    Vx[15+(NY) * 0] = 0;
    Vx[15+(NY) * 1] = 0;
    Vx[15+(NY) * 2] = 0;
    Vx[15+(NY) * 3] = 0;
    Vx[15+(NY) * 4] = 0;
    Vx[15+(NY) * 5] = 0;
    Vx[15+(NY) * 6] = 0;
    Vx[15+(NY) * 7] = 0;
    Vx[15+(NY) * 8] = 0;
    Vx[15+(NY) * 9] = 0;
    Vx[15+(NY) * 10] = 0;
    Vx[15+(NY) * 11] = 0;
    Vx[15+(NY) * 12] = 0;
    Vx[15+(NY) * 13] = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }


    double Vu[NY*NU];
    
    Vu[0+(NY) * 0] = 0;
    Vu[0+(NY) * 1] = 0;
    Vu[1+(NY) * 0] = 0;
    Vu[1+(NY) * 1] = 0;
    Vu[2+(NY) * 0] = 0;
    Vu[2+(NY) * 1] = 0;
    Vu[3+(NY) * 0] = 0;
    Vu[3+(NY) * 1] = 0;
    Vu[4+(NY) * 0] = 0;
    Vu[4+(NY) * 1] = 0;
    Vu[5+(NY) * 0] = 0;
    Vu[5+(NY) * 1] = 0;
    Vu[6+(NY) * 0] = 0;
    Vu[6+(NY) * 1] = 0;
    Vu[7+(NY) * 0] = 0;
    Vu[7+(NY) * 1] = 0;
    Vu[8+(NY) * 0] = 0;
    Vu[8+(NY) * 1] = 0;
    Vu[9+(NY) * 0] = 0;
    Vu[9+(NY) * 1] = 0;
    Vu[10+(NY) * 0] = 0;
    Vu[10+(NY) * 1] = 0;
    Vu[11+(NY) * 0] = 0;
    Vu[11+(NY) * 1] = 0;
    Vu[12+(NY) * 0] = 0;
    Vu[12+(NY) * 1] = 0;
    Vu[13+(NY) * 0] = 0;
    Vu[13+(NY) * 1] = 0;
    Vu[14+(NY) * 0] = 1;
    Vu[14+(NY) * 1] = 0;
    Vu[15+(NY) * 0] = 0;
    Vu[15+(NY) * 1] = 1;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }







    // terminal cost


    double yref_e[NYN];
    
    yref_e[0] = 0;
    yref_e[1] = 0;
    yref_e[2] = 0;
    yref_e[3] = 0;
    yref_e[4] = 0;
    yref_e[5] = 0;
    yref_e[6] = 0;
    yref_e[7] = 0;
    yref_e[8] = 0;
    yref_e[9] = 0;
    yref_e[10] = 0;
    yref_e[11] = 0;
    yref_e[12] = 0;
    yref_e[13] = 0;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);

    double W_e[NYN*NYN];
    
    W_e[0+(NYN) * 0] = 0;
    W_e[0+(NYN) * 1] = 0;
    W_e[0+(NYN) * 2] = 0;
    W_e[0+(NYN) * 3] = 0;
    W_e[0+(NYN) * 4] = 0;
    W_e[0+(NYN) * 5] = 0;
    W_e[0+(NYN) * 6] = 0;
    W_e[0+(NYN) * 7] = 0;
    W_e[0+(NYN) * 8] = 0;
    W_e[0+(NYN) * 9] = 0;
    W_e[0+(NYN) * 10] = 0;
    W_e[0+(NYN) * 11] = 0;
    W_e[0+(NYN) * 12] = 0;
    W_e[0+(NYN) * 13] = 0;
    W_e[1+(NYN) * 0] = 0;
    W_e[1+(NYN) * 1] = 0.5;
    W_e[1+(NYN) * 2] = 0;
    W_e[1+(NYN) * 3] = 0;
    W_e[1+(NYN) * 4] = 0;
    W_e[1+(NYN) * 5] = 0;
    W_e[1+(NYN) * 6] = 0;
    W_e[1+(NYN) * 7] = 0;
    W_e[1+(NYN) * 8] = 0;
    W_e[1+(NYN) * 9] = 0;
    W_e[1+(NYN) * 10] = 0;
    W_e[1+(NYN) * 11] = 0;
    W_e[1+(NYN) * 12] = 0;
    W_e[1+(NYN) * 13] = 0;
    W_e[2+(NYN) * 0] = 0;
    W_e[2+(NYN) * 1] = 0;
    W_e[2+(NYN) * 2] = 0.5;
    W_e[2+(NYN) * 3] = 0;
    W_e[2+(NYN) * 4] = 0;
    W_e[2+(NYN) * 5] = 0;
    W_e[2+(NYN) * 6] = 0;
    W_e[2+(NYN) * 7] = 0;
    W_e[2+(NYN) * 8] = 0;
    W_e[2+(NYN) * 9] = 0;
    W_e[2+(NYN) * 10] = 0;
    W_e[2+(NYN) * 11] = 0;
    W_e[2+(NYN) * 12] = 0;
    W_e[2+(NYN) * 13] = 0;
    W_e[3+(NYN) * 0] = 0;
    W_e[3+(NYN) * 1] = 0;
    W_e[3+(NYN) * 2] = 0;
    W_e[3+(NYN) * 3] = 100;
    W_e[3+(NYN) * 4] = 0;
    W_e[3+(NYN) * 5] = 0;
    W_e[3+(NYN) * 6] = 0;
    W_e[3+(NYN) * 7] = 0;
    W_e[3+(NYN) * 8] = 0;
    W_e[3+(NYN) * 9] = 0;
    W_e[3+(NYN) * 10] = 0;
    W_e[3+(NYN) * 11] = 0;
    W_e[3+(NYN) * 12] = 0;
    W_e[3+(NYN) * 13] = 0;
    W_e[4+(NYN) * 0] = 0;
    W_e[4+(NYN) * 1] = 0;
    W_e[4+(NYN) * 2] = 0;
    W_e[4+(NYN) * 3] = 0;
    W_e[4+(NYN) * 4] = 0;
    W_e[4+(NYN) * 5] = 0;
    W_e[4+(NYN) * 6] = 0;
    W_e[4+(NYN) * 7] = 0;
    W_e[4+(NYN) * 8] = 0;
    W_e[4+(NYN) * 9] = 0;
    W_e[4+(NYN) * 10] = 0;
    W_e[4+(NYN) * 11] = 0;
    W_e[4+(NYN) * 12] = 0;
    W_e[4+(NYN) * 13] = 0;
    W_e[5+(NYN) * 0] = 0;
    W_e[5+(NYN) * 1] = 0;
    W_e[5+(NYN) * 2] = 0;
    W_e[5+(NYN) * 3] = 0;
    W_e[5+(NYN) * 4] = 0;
    W_e[5+(NYN) * 5] = 0;
    W_e[5+(NYN) * 6] = 0;
    W_e[5+(NYN) * 7] = 0;
    W_e[5+(NYN) * 8] = 0;
    W_e[5+(NYN) * 9] = 0;
    W_e[5+(NYN) * 10] = 0;
    W_e[5+(NYN) * 11] = 0;
    W_e[5+(NYN) * 12] = 0;
    W_e[5+(NYN) * 13] = 0;
    W_e[6+(NYN) * 0] = 0;
    W_e[6+(NYN) * 1] = 0;
    W_e[6+(NYN) * 2] = 0;
    W_e[6+(NYN) * 3] = 0;
    W_e[6+(NYN) * 4] = 0;
    W_e[6+(NYN) * 5] = 0;
    W_e[6+(NYN) * 6] = 1;
    W_e[6+(NYN) * 7] = 0;
    W_e[6+(NYN) * 8] = 0;
    W_e[6+(NYN) * 9] = 0;
    W_e[6+(NYN) * 10] = 0;
    W_e[6+(NYN) * 11] = 0;
    W_e[6+(NYN) * 12] = 0;
    W_e[6+(NYN) * 13] = 0;
    W_e[7+(NYN) * 0] = 0;
    W_e[7+(NYN) * 1] = 0;
    W_e[7+(NYN) * 2] = 0;
    W_e[7+(NYN) * 3] = 0;
    W_e[7+(NYN) * 4] = 0;
    W_e[7+(NYN) * 5] = 0;
    W_e[7+(NYN) * 6] = 0;
    W_e[7+(NYN) * 7] = 0;
    W_e[7+(NYN) * 8] = 0;
    W_e[7+(NYN) * 9] = 0;
    W_e[7+(NYN) * 10] = 0;
    W_e[7+(NYN) * 11] = 0;
    W_e[7+(NYN) * 12] = 0;
    W_e[7+(NYN) * 13] = 0;
    W_e[8+(NYN) * 0] = 0;
    W_e[8+(NYN) * 1] = 0;
    W_e[8+(NYN) * 2] = 0;
    W_e[8+(NYN) * 3] = 0;
    W_e[8+(NYN) * 4] = 0;
    W_e[8+(NYN) * 5] = 0;
    W_e[8+(NYN) * 6] = 0;
    W_e[8+(NYN) * 7] = 0;
    W_e[8+(NYN) * 8] = 0;
    W_e[8+(NYN) * 9] = 0;
    W_e[8+(NYN) * 10] = 0;
    W_e[8+(NYN) * 11] = 0;
    W_e[8+(NYN) * 12] = 0;
    W_e[8+(NYN) * 13] = 0;
    W_e[9+(NYN) * 0] = 0;
    W_e[9+(NYN) * 1] = 0;
    W_e[9+(NYN) * 2] = 0;
    W_e[9+(NYN) * 3] = 0;
    W_e[9+(NYN) * 4] = 0;
    W_e[9+(NYN) * 5] = 0;
    W_e[9+(NYN) * 6] = 0;
    W_e[9+(NYN) * 7] = 0;
    W_e[9+(NYN) * 8] = 0;
    W_e[9+(NYN) * 9] = 0;
    W_e[9+(NYN) * 10] = 0;
    W_e[9+(NYN) * 11] = 0;
    W_e[9+(NYN) * 12] = 0;
    W_e[9+(NYN) * 13] = 0;
    W_e[10+(NYN) * 0] = 0;
    W_e[10+(NYN) * 1] = 0;
    W_e[10+(NYN) * 2] = 0;
    W_e[10+(NYN) * 3] = 0;
    W_e[10+(NYN) * 4] = 0;
    W_e[10+(NYN) * 5] = 0;
    W_e[10+(NYN) * 6] = 0;
    W_e[10+(NYN) * 7] = 0;
    W_e[10+(NYN) * 8] = 0;
    W_e[10+(NYN) * 9] = 0;
    W_e[10+(NYN) * 10] = 0;
    W_e[10+(NYN) * 11] = 0;
    W_e[10+(NYN) * 12] = 0;
    W_e[10+(NYN) * 13] = 0;
    W_e[11+(NYN) * 0] = 0;
    W_e[11+(NYN) * 1] = 0;
    W_e[11+(NYN) * 2] = 0;
    W_e[11+(NYN) * 3] = 0;
    W_e[11+(NYN) * 4] = 0;
    W_e[11+(NYN) * 5] = 0;
    W_e[11+(NYN) * 6] = 0;
    W_e[11+(NYN) * 7] = 0;
    W_e[11+(NYN) * 8] = 0;
    W_e[11+(NYN) * 9] = 0;
    W_e[11+(NYN) * 10] = 0;
    W_e[11+(NYN) * 11] = 0;
    W_e[11+(NYN) * 12] = 0;
    W_e[11+(NYN) * 13] = 0;
    W_e[12+(NYN) * 0] = 0;
    W_e[12+(NYN) * 1] = 0;
    W_e[12+(NYN) * 2] = 0;
    W_e[12+(NYN) * 3] = 0;
    W_e[12+(NYN) * 4] = 0;
    W_e[12+(NYN) * 5] = 0;
    W_e[12+(NYN) * 6] = 0;
    W_e[12+(NYN) * 7] = 0;
    W_e[12+(NYN) * 8] = 0;
    W_e[12+(NYN) * 9] = 0;
    W_e[12+(NYN) * 10] = 0;
    W_e[12+(NYN) * 11] = 0;
    W_e[12+(NYN) * 12] = 0.0005;
    W_e[12+(NYN) * 13] = 0;
    W_e[13+(NYN) * 0] = 0;
    W_e[13+(NYN) * 1] = 0;
    W_e[13+(NYN) * 2] = 0;
    W_e[13+(NYN) * 3] = 0;
    W_e[13+(NYN) * 4] = 0;
    W_e[13+(NYN) * 5] = 0;
    W_e[13+(NYN) * 6] = 0;
    W_e[13+(NYN) * 7] = 0;
    W_e[13+(NYN) * 8] = 0;
    W_e[13+(NYN) * 9] = 0;
    W_e[13+(NYN) * 10] = 0;
    W_e[13+(NYN) * 11] = 0;
    W_e[13+(NYN) * 12] = 0;
    W_e[13+(NYN) * 13] = 0.0005;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    double Vx_e[NYN*NX];
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[0+(NYN) * 1] = 0;
    Vx_e[0+(NYN) * 2] = 0;
    Vx_e[0+(NYN) * 3] = 0;
    Vx_e[0+(NYN) * 4] = 0;
    Vx_e[0+(NYN) * 5] = 0;
    Vx_e[0+(NYN) * 6] = 0;
    Vx_e[0+(NYN) * 7] = 0;
    Vx_e[0+(NYN) * 8] = 0;
    Vx_e[0+(NYN) * 9] = 0;
    Vx_e[0+(NYN) * 10] = 0;
    Vx_e[0+(NYN) * 11] = 0;
    Vx_e[0+(NYN) * 12] = 0;
    Vx_e[0+(NYN) * 13] = 0;
    Vx_e[1+(NYN) * 0] = 0;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[1+(NYN) * 2] = 0;
    Vx_e[1+(NYN) * 3] = 0;
    Vx_e[1+(NYN) * 4] = 0;
    Vx_e[1+(NYN) * 5] = 0;
    Vx_e[1+(NYN) * 6] = 0;
    Vx_e[1+(NYN) * 7] = 0;
    Vx_e[1+(NYN) * 8] = 0;
    Vx_e[1+(NYN) * 9] = 0;
    Vx_e[1+(NYN) * 10] = 0;
    Vx_e[1+(NYN) * 11] = 0;
    Vx_e[1+(NYN) * 12] = 0;
    Vx_e[1+(NYN) * 13] = 0;
    Vx_e[2+(NYN) * 0] = 0;
    Vx_e[2+(NYN) * 1] = 0;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[2+(NYN) * 3] = 0;
    Vx_e[2+(NYN) * 4] = 0;
    Vx_e[2+(NYN) * 5] = 0;
    Vx_e[2+(NYN) * 6] = 0;
    Vx_e[2+(NYN) * 7] = 0;
    Vx_e[2+(NYN) * 8] = 0;
    Vx_e[2+(NYN) * 9] = 0;
    Vx_e[2+(NYN) * 10] = 0;
    Vx_e[2+(NYN) * 11] = 0;
    Vx_e[2+(NYN) * 12] = 0;
    Vx_e[2+(NYN) * 13] = 0;
    Vx_e[3+(NYN) * 0] = 0;
    Vx_e[3+(NYN) * 1] = 0;
    Vx_e[3+(NYN) * 2] = 0;
    Vx_e[3+(NYN) * 3] = 1;
    Vx_e[3+(NYN) * 4] = 0;
    Vx_e[3+(NYN) * 5] = 0;
    Vx_e[3+(NYN) * 6] = 0;
    Vx_e[3+(NYN) * 7] = 0;
    Vx_e[3+(NYN) * 8] = 0;
    Vx_e[3+(NYN) * 9] = 0;
    Vx_e[3+(NYN) * 10] = 0;
    Vx_e[3+(NYN) * 11] = 0;
    Vx_e[3+(NYN) * 12] = 0;
    Vx_e[3+(NYN) * 13] = 0;
    Vx_e[4+(NYN) * 0] = 0;
    Vx_e[4+(NYN) * 1] = 0;
    Vx_e[4+(NYN) * 2] = 0;
    Vx_e[4+(NYN) * 3] = 0;
    Vx_e[4+(NYN) * 4] = 1;
    Vx_e[4+(NYN) * 5] = 0;
    Vx_e[4+(NYN) * 6] = 0;
    Vx_e[4+(NYN) * 7] = 0;
    Vx_e[4+(NYN) * 8] = 0;
    Vx_e[4+(NYN) * 9] = 0;
    Vx_e[4+(NYN) * 10] = 0;
    Vx_e[4+(NYN) * 11] = 0;
    Vx_e[4+(NYN) * 12] = 0;
    Vx_e[4+(NYN) * 13] = 0;
    Vx_e[5+(NYN) * 0] = 0;
    Vx_e[5+(NYN) * 1] = 0;
    Vx_e[5+(NYN) * 2] = 0;
    Vx_e[5+(NYN) * 3] = 0;
    Vx_e[5+(NYN) * 4] = 0;
    Vx_e[5+(NYN) * 5] = 1;
    Vx_e[5+(NYN) * 6] = 0;
    Vx_e[5+(NYN) * 7] = 0;
    Vx_e[5+(NYN) * 8] = 0;
    Vx_e[5+(NYN) * 9] = 0;
    Vx_e[5+(NYN) * 10] = 0;
    Vx_e[5+(NYN) * 11] = 0;
    Vx_e[5+(NYN) * 12] = 0;
    Vx_e[5+(NYN) * 13] = 0;
    Vx_e[6+(NYN) * 0] = 0;
    Vx_e[6+(NYN) * 1] = 0;
    Vx_e[6+(NYN) * 2] = 0;
    Vx_e[6+(NYN) * 3] = 0;
    Vx_e[6+(NYN) * 4] = 0;
    Vx_e[6+(NYN) * 5] = 0;
    Vx_e[6+(NYN) * 6] = 1;
    Vx_e[6+(NYN) * 7] = 0;
    Vx_e[6+(NYN) * 8] = 0;
    Vx_e[6+(NYN) * 9] = 0;
    Vx_e[6+(NYN) * 10] = 0;
    Vx_e[6+(NYN) * 11] = 0;
    Vx_e[6+(NYN) * 12] = 0;
    Vx_e[6+(NYN) * 13] = 0;
    Vx_e[7+(NYN) * 0] = 0;
    Vx_e[7+(NYN) * 1] = 0;
    Vx_e[7+(NYN) * 2] = 0;
    Vx_e[7+(NYN) * 3] = 0;
    Vx_e[7+(NYN) * 4] = 0;
    Vx_e[7+(NYN) * 5] = 0;
    Vx_e[7+(NYN) * 6] = 0;
    Vx_e[7+(NYN) * 7] = 1;
    Vx_e[7+(NYN) * 8] = 0;
    Vx_e[7+(NYN) * 9] = 0;
    Vx_e[7+(NYN) * 10] = 0;
    Vx_e[7+(NYN) * 11] = 0;
    Vx_e[7+(NYN) * 12] = 0;
    Vx_e[7+(NYN) * 13] = 0;
    Vx_e[8+(NYN) * 0] = 0;
    Vx_e[8+(NYN) * 1] = 0;
    Vx_e[8+(NYN) * 2] = 0;
    Vx_e[8+(NYN) * 3] = 0;
    Vx_e[8+(NYN) * 4] = 0;
    Vx_e[8+(NYN) * 5] = 0;
    Vx_e[8+(NYN) * 6] = 0;
    Vx_e[8+(NYN) * 7] = 0;
    Vx_e[8+(NYN) * 8] = 1;
    Vx_e[8+(NYN) * 9] = 0;
    Vx_e[8+(NYN) * 10] = 0;
    Vx_e[8+(NYN) * 11] = 0;
    Vx_e[8+(NYN) * 12] = 0;
    Vx_e[8+(NYN) * 13] = 0;
    Vx_e[9+(NYN) * 0] = 0;
    Vx_e[9+(NYN) * 1] = 0;
    Vx_e[9+(NYN) * 2] = 0;
    Vx_e[9+(NYN) * 3] = 0;
    Vx_e[9+(NYN) * 4] = 0;
    Vx_e[9+(NYN) * 5] = 0;
    Vx_e[9+(NYN) * 6] = 0;
    Vx_e[9+(NYN) * 7] = 0;
    Vx_e[9+(NYN) * 8] = 0;
    Vx_e[9+(NYN) * 9] = 1;
    Vx_e[9+(NYN) * 10] = 0;
    Vx_e[9+(NYN) * 11] = 0;
    Vx_e[9+(NYN) * 12] = 0;
    Vx_e[9+(NYN) * 13] = 0;
    Vx_e[10+(NYN) * 0] = 0;
    Vx_e[10+(NYN) * 1] = 0;
    Vx_e[10+(NYN) * 2] = 0;
    Vx_e[10+(NYN) * 3] = 0;
    Vx_e[10+(NYN) * 4] = 0;
    Vx_e[10+(NYN) * 5] = 0;
    Vx_e[10+(NYN) * 6] = 0;
    Vx_e[10+(NYN) * 7] = 0;
    Vx_e[10+(NYN) * 8] = 0;
    Vx_e[10+(NYN) * 9] = 0;
    Vx_e[10+(NYN) * 10] = 1;
    Vx_e[10+(NYN) * 11] = 0;
    Vx_e[10+(NYN) * 12] = 0;
    Vx_e[10+(NYN) * 13] = 0;
    Vx_e[11+(NYN) * 0] = 0;
    Vx_e[11+(NYN) * 1] = 0;
    Vx_e[11+(NYN) * 2] = 0;
    Vx_e[11+(NYN) * 3] = 0;
    Vx_e[11+(NYN) * 4] = 0;
    Vx_e[11+(NYN) * 5] = 0;
    Vx_e[11+(NYN) * 6] = 0;
    Vx_e[11+(NYN) * 7] = 0;
    Vx_e[11+(NYN) * 8] = 0;
    Vx_e[11+(NYN) * 9] = 0;
    Vx_e[11+(NYN) * 10] = 0;
    Vx_e[11+(NYN) * 11] = 1;
    Vx_e[11+(NYN) * 12] = 0;
    Vx_e[11+(NYN) * 13] = 0;
    Vx_e[12+(NYN) * 0] = 0;
    Vx_e[12+(NYN) * 1] = 0;
    Vx_e[12+(NYN) * 2] = 0;
    Vx_e[12+(NYN) * 3] = 0;
    Vx_e[12+(NYN) * 4] = 0;
    Vx_e[12+(NYN) * 5] = 0;
    Vx_e[12+(NYN) * 6] = 0;
    Vx_e[12+(NYN) * 7] = 0;
    Vx_e[12+(NYN) * 8] = 0;
    Vx_e[12+(NYN) * 9] = 0;
    Vx_e[12+(NYN) * 10] = 0;
    Vx_e[12+(NYN) * 11] = 0;
    Vx_e[12+(NYN) * 12] = 1;
    Vx_e[12+(NYN) * 13] = 0;
    Vx_e[13+(NYN) * 0] = 0;
    Vx_e[13+(NYN) * 1] = 0;
    Vx_e[13+(NYN) * 2] = 0;
    Vx_e[13+(NYN) * 3] = 0;
    Vx_e[13+(NYN) * 4] = 0;
    Vx_e[13+(NYN) * 5] = 0;
    Vx_e[13+(NYN) * 6] = 0;
    Vx_e[13+(NYN) * 7] = 0;
    Vx_e[13+(NYN) * 8] = 0;
    Vx_e[13+(NYN) * 9] = 0;
    Vx_e[13+(NYN) * 10] = 0;
    Vx_e[13+(NYN) * 11] = 0;
    Vx_e[13+(NYN) * 12] = 0;
    Vx_e[13+(NYN) * 13] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);



    /**** Constraints ****/

    // bounds for initial stage

    // x0
    int idxbx0[14];
    
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;

    double lbx0[14];
    double ubx0[14];
    
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 1;
    ubx0[2] = 1;
    lbx0[3] = 0.001;
    ubx0[3] = 0.001;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;
    lbx0[6] = 0;
    ubx0[6] = 0;
    lbx0[7] = 1;
    ubx0[7] = 1;
    lbx0[8] = -1;
    ubx0[8] = -1;
    lbx0[9] = 1.5707963267948966;
    ubx0[9] = 1.5707963267948966;
    lbx0[10] = 0;
    ubx0[10] = 0;
    lbx0[11] = 0;
    ubx0[11] = 0;
    lbx0[12] = 0;
    ubx0[12] = 0;
    lbx0[13] = 0;
    ubx0[13] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);


    // idxbxe_0
    int idxbxe_0[14];
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);


    /* constraints that are the same for initial and intermediate */



    // u
    int idxbu[NBU];
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    double lbu[NBU];
    double ubu[NBU];
    
    lbu[0] = -30;
    ubu[0] = 30;
    lbu[1] = -30;
    ubu[1] = 30;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }











    // x
    int idxbx[NBX];
    
    idxbx[0] = 3;
    idxbx[1] = 4;
    idxbx[2] = 5;
    idxbx[3] = 12;
    idxbx[4] = 13;
    double lbx[NBX];
    double ubx[NBX];
    
    lbx[0] = -2;
    ubx[0] = 2;
    lbx[1] = -2;
    ubx[1] = 2;
    lbx[2] = -10;
    ubx[2] = 10;
    lbx[3] = -30;
    ubx[3] = 35;
    lbx[4] = -30;
    ubx[4] = 35;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }








    /* terminal constraints */

















    /************************************************
    *  opts
    ************************************************/

    nlp_opts = ocp_nlp_solver_opts_create(nlp_config, nlp_dims);


    int num_steps_val = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &num_steps_val);

    int ns_val = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &ns_val);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    bool tmp_bool = false;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;
    // NOTE: there is no condensing happening here!
    qp_solver_cond_N = N;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);


    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);


    int ext_cost_num_hess = 0;


    /* out */
    nlp_out = ocp_nlp_out_create(nlp_config, nlp_dims);

    // initialize primal solution
    double x0[14];

    // initialize with x0
    
    x0[0] = 0;
    x0[1] = 0;
    x0[2] = 1;
    x0[3] = 0.001;
    x0[4] = 0;
    x0[5] = 0;
    x0[6] = 0;
    x0[7] = 1;
    x0[8] = -1;
    x0[9] = 1.5707963267948966;
    x0[10] = 0;
    x0[11] = 0;
    x0[12] = 0;
    x0[13] = 0;


    double u0[NU];
    
    u0[0] = 0.0;
    u0[1] = 0.0;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    
    nlp_solver = ocp_nlp_solver_create(nlp_config, nlp_dims, nlp_opts);




    status = ocp_nlp_precompute(nlp_solver, nlp_in, nlp_out);

    if (status != ACADOS_SUCCESS)
    {
        printf("\nocp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int acados_update_params(int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    return solver_status;
}



int acados_solve()
{
    // solve NLP 
    int solver_status = ocp_nlp_solve(nlp_solver, nlp_in, nlp_out);

    return solver_status;
}


int acados_free()
{
    // free memory
    ocp_nlp_solver_opts_destroy(nlp_opts);
    ocp_nlp_in_destroy(nlp_in);
    ocp_nlp_out_destroy(nlp_out);
    ocp_nlp_solver_destroy(nlp_solver);
    ocp_nlp_dims_destroy(nlp_dims);
    ocp_nlp_config_destroy(nlp_config);
    ocp_nlp_plan_destroy(nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < 100; i++)
    {
        external_function_param_casadi_free(&forw_vde_casadi[i]);
        external_function_param_casadi_free(&expl_ode_fun[i]);
    }
    free(forw_vde_casadi);
    free(expl_ode_fun);

    // cost

    // constraints

    return 0;
}

ocp_nlp_in * acados_get_nlp_in() { return  nlp_in; }
ocp_nlp_out * acados_get_nlp_out() { return  nlp_out; }
ocp_nlp_solver * acados_get_nlp_solver() { return  nlp_solver; }
ocp_nlp_config * acados_get_nlp_config() { return  nlp_config; }
void * acados_get_nlp_opts() { return  nlp_opts; }
ocp_nlp_dims * acados_get_nlp_dims() { return  nlp_dims; }
ocp_nlp_plan * acados_get_nlp_plan() { return  nlp_solver_plan; }


void acados_print_stats()
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(nlp_config, nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(nlp_config, nlp_solver, "stat_m", &stat_m);

    
    double stat[1000];
    ocp_nlp_get(nlp_config, nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j > 4)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}
