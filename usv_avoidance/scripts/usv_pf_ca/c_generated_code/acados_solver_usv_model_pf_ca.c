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
#include "usv_model_pf_ca_model/usv_model_pf_ca_model.h"



#include "usv_model_pf_ca_constraints/usv_model_pf_ca_h_constraint.h"



#include "acados_solver_usv_model_pf_ca.h"

#define NX     12
#define NZ     0
#define NU     2
#define NP     16
#define NBX    5
#define NBX0   12
#define NBU    2
#define NSBX   0
#define NSBU   0
#define NSH    8
#define NSG    0
#define NSPHI  0
#define NSHN   0
#define NSGN   0
#define NSPHIN 0
#define NSBXN  0
#define NS     8
#define NSN    0
#define NG     0
#define NBXN   0
#define NGN    0
#define NY     14
#define NYN    12
#define N      400
#define NH     8
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
    nbxe[0] = 12;

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
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);



    /************************************************
    *  external functions
    ************************************************/
    nl_constr_h_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        nl_constr_h_fun_jac[i].casadi_fun = &usv_model_pf_ca_constr_h_fun_jac_uxt_zt;
        nl_constr_h_fun_jac[i].casadi_n_in = &usv_model_pf_ca_constr_h_fun_jac_uxt_zt_n_in;
        nl_constr_h_fun_jac[i].casadi_n_out = &usv_model_pf_ca_constr_h_fun_jac_uxt_zt_n_out;
        nl_constr_h_fun_jac[i].casadi_sparsity_in = &usv_model_pf_ca_constr_h_fun_jac_uxt_zt_sparsity_in;
        nl_constr_h_fun_jac[i].casadi_sparsity_out = &usv_model_pf_ca_constr_h_fun_jac_uxt_zt_sparsity_out;
        nl_constr_h_fun_jac[i].casadi_work = &usv_model_pf_ca_constr_h_fun_jac_uxt_zt_work;
        external_function_param_casadi_create(&nl_constr_h_fun_jac[i], 16);
    }
    nl_constr_h_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        nl_constr_h_fun[i].casadi_fun = &usv_model_pf_ca_constr_h_fun;
        nl_constr_h_fun[i].casadi_n_in = &usv_model_pf_ca_constr_h_fun_n_in;
        nl_constr_h_fun[i].casadi_n_out = &usv_model_pf_ca_constr_h_fun_n_out;
        nl_constr_h_fun[i].casadi_sparsity_in = &usv_model_pf_ca_constr_h_fun_sparsity_in;
        nl_constr_h_fun[i].casadi_sparsity_out = &usv_model_pf_ca_constr_h_fun_sparsity_out;
        nl_constr_h_fun[i].casadi_work = &usv_model_pf_ca_constr_h_fun_work;
        external_function_param_casadi_create(&nl_constr_h_fun[i], 16);
    }
    
    


    // explicit ode
    forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        forw_vde_casadi[i].casadi_fun = &usv_model_pf_ca_expl_vde_forw;
        forw_vde_casadi[i].casadi_n_in = &usv_model_pf_ca_expl_vde_forw_n_in;
        forw_vde_casadi[i].casadi_n_out = &usv_model_pf_ca_expl_vde_forw_n_out;
        forw_vde_casadi[i].casadi_sparsity_in = &usv_model_pf_ca_expl_vde_forw_sparsity_in;
        forw_vde_casadi[i].casadi_sparsity_out = &usv_model_pf_ca_expl_vde_forw_sparsity_out;
        forw_vde_casadi[i].casadi_work = &usv_model_pf_ca_expl_vde_forw_work;
        external_function_param_casadi_create(&forw_vde_casadi[i], 16);
    }

    expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        expl_ode_fun[i].casadi_fun = &usv_model_pf_ca_expl_ode_fun;
        expl_ode_fun[i].casadi_n_in = &usv_model_pf_ca_expl_ode_fun_n_in;
        expl_ode_fun[i].casadi_n_out = &usv_model_pf_ca_expl_ode_fun_n_out;
        expl_ode_fun[i].casadi_sparsity_in = &usv_model_pf_ca_expl_ode_fun_sparsity_in;
        expl_ode_fun[i].casadi_sparsity_out = &usv_model_pf_ca_expl_ode_fun_sparsity_out;
        expl_ode_fun[i].casadi_work = &usv_model_pf_ca_expl_ode_fun_work;
        external_function_param_casadi_create(&expl_ode_fun[i], 16);
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
    time_steps[100] = 0.01;
    time_steps[101] = 0.01;
    time_steps[102] = 0.01;
    time_steps[103] = 0.01;
    time_steps[104] = 0.01;
    time_steps[105] = 0.01;
    time_steps[106] = 0.01;
    time_steps[107] = 0.01;
    time_steps[108] = 0.01;
    time_steps[109] = 0.01;
    time_steps[110] = 0.01;
    time_steps[111] = 0.01;
    time_steps[112] = 0.01;
    time_steps[113] = 0.01;
    time_steps[114] = 0.01;
    time_steps[115] = 0.01;
    time_steps[116] = 0.01;
    time_steps[117] = 0.01;
    time_steps[118] = 0.01;
    time_steps[119] = 0.01;
    time_steps[120] = 0.01;
    time_steps[121] = 0.01;
    time_steps[122] = 0.01;
    time_steps[123] = 0.01;
    time_steps[124] = 0.01;
    time_steps[125] = 0.01;
    time_steps[126] = 0.01;
    time_steps[127] = 0.01;
    time_steps[128] = 0.01;
    time_steps[129] = 0.01;
    time_steps[130] = 0.01;
    time_steps[131] = 0.01;
    time_steps[132] = 0.01;
    time_steps[133] = 0.01;
    time_steps[134] = 0.01;
    time_steps[135] = 0.01;
    time_steps[136] = 0.01;
    time_steps[137] = 0.01;
    time_steps[138] = 0.01;
    time_steps[139] = 0.01;
    time_steps[140] = 0.01;
    time_steps[141] = 0.01;
    time_steps[142] = 0.01;
    time_steps[143] = 0.01;
    time_steps[144] = 0.01;
    time_steps[145] = 0.01;
    time_steps[146] = 0.01;
    time_steps[147] = 0.01;
    time_steps[148] = 0.01;
    time_steps[149] = 0.01;
    time_steps[150] = 0.01;
    time_steps[151] = 0.01;
    time_steps[152] = 0.01;
    time_steps[153] = 0.01;
    time_steps[154] = 0.01;
    time_steps[155] = 0.01;
    time_steps[156] = 0.01;
    time_steps[157] = 0.01;
    time_steps[158] = 0.01;
    time_steps[159] = 0.01;
    time_steps[160] = 0.01;
    time_steps[161] = 0.01;
    time_steps[162] = 0.01;
    time_steps[163] = 0.01;
    time_steps[164] = 0.01;
    time_steps[165] = 0.01;
    time_steps[166] = 0.01;
    time_steps[167] = 0.01;
    time_steps[168] = 0.01;
    time_steps[169] = 0.01;
    time_steps[170] = 0.01;
    time_steps[171] = 0.01;
    time_steps[172] = 0.01;
    time_steps[173] = 0.01;
    time_steps[174] = 0.01;
    time_steps[175] = 0.01;
    time_steps[176] = 0.01;
    time_steps[177] = 0.01;
    time_steps[178] = 0.01;
    time_steps[179] = 0.01;
    time_steps[180] = 0.01;
    time_steps[181] = 0.01;
    time_steps[182] = 0.01;
    time_steps[183] = 0.01;
    time_steps[184] = 0.01;
    time_steps[185] = 0.01;
    time_steps[186] = 0.01;
    time_steps[187] = 0.01;
    time_steps[188] = 0.01;
    time_steps[189] = 0.01;
    time_steps[190] = 0.01;
    time_steps[191] = 0.01;
    time_steps[192] = 0.01;
    time_steps[193] = 0.01;
    time_steps[194] = 0.01;
    time_steps[195] = 0.01;
    time_steps[196] = 0.01;
    time_steps[197] = 0.01;
    time_steps[198] = 0.01;
    time_steps[199] = 0.01;
    time_steps[200] = 0.01;
    time_steps[201] = 0.01;
    time_steps[202] = 0.01;
    time_steps[203] = 0.01;
    time_steps[204] = 0.01;
    time_steps[205] = 0.01;
    time_steps[206] = 0.01;
    time_steps[207] = 0.01;
    time_steps[208] = 0.01;
    time_steps[209] = 0.01;
    time_steps[210] = 0.01;
    time_steps[211] = 0.01;
    time_steps[212] = 0.01;
    time_steps[213] = 0.01;
    time_steps[214] = 0.01;
    time_steps[215] = 0.01;
    time_steps[216] = 0.01;
    time_steps[217] = 0.01;
    time_steps[218] = 0.01;
    time_steps[219] = 0.01;
    time_steps[220] = 0.01;
    time_steps[221] = 0.01;
    time_steps[222] = 0.01;
    time_steps[223] = 0.01;
    time_steps[224] = 0.01;
    time_steps[225] = 0.01;
    time_steps[226] = 0.01;
    time_steps[227] = 0.01;
    time_steps[228] = 0.01;
    time_steps[229] = 0.01;
    time_steps[230] = 0.01;
    time_steps[231] = 0.01;
    time_steps[232] = 0.01;
    time_steps[233] = 0.01;
    time_steps[234] = 0.01;
    time_steps[235] = 0.01;
    time_steps[236] = 0.01;
    time_steps[237] = 0.01;
    time_steps[238] = 0.01;
    time_steps[239] = 0.01;
    time_steps[240] = 0.01;
    time_steps[241] = 0.01;
    time_steps[242] = 0.01;
    time_steps[243] = 0.01;
    time_steps[244] = 0.01;
    time_steps[245] = 0.01;
    time_steps[246] = 0.01;
    time_steps[247] = 0.01;
    time_steps[248] = 0.01;
    time_steps[249] = 0.01;
    time_steps[250] = 0.01;
    time_steps[251] = 0.01;
    time_steps[252] = 0.01;
    time_steps[253] = 0.01;
    time_steps[254] = 0.01;
    time_steps[255] = 0.01;
    time_steps[256] = 0.01;
    time_steps[257] = 0.01;
    time_steps[258] = 0.01;
    time_steps[259] = 0.01;
    time_steps[260] = 0.01;
    time_steps[261] = 0.01;
    time_steps[262] = 0.01;
    time_steps[263] = 0.01;
    time_steps[264] = 0.01;
    time_steps[265] = 0.01;
    time_steps[266] = 0.01;
    time_steps[267] = 0.01;
    time_steps[268] = 0.01;
    time_steps[269] = 0.01;
    time_steps[270] = 0.01;
    time_steps[271] = 0.01;
    time_steps[272] = 0.01;
    time_steps[273] = 0.01;
    time_steps[274] = 0.01;
    time_steps[275] = 0.01;
    time_steps[276] = 0.01;
    time_steps[277] = 0.01;
    time_steps[278] = 0.01;
    time_steps[279] = 0.01;
    time_steps[280] = 0.01;
    time_steps[281] = 0.01;
    time_steps[282] = 0.01;
    time_steps[283] = 0.01;
    time_steps[284] = 0.01;
    time_steps[285] = 0.01;
    time_steps[286] = 0.01;
    time_steps[287] = 0.01;
    time_steps[288] = 0.01;
    time_steps[289] = 0.01;
    time_steps[290] = 0.01;
    time_steps[291] = 0.01;
    time_steps[292] = 0.01;
    time_steps[293] = 0.01;
    time_steps[294] = 0.01;
    time_steps[295] = 0.01;
    time_steps[296] = 0.01;
    time_steps[297] = 0.01;
    time_steps[298] = 0.01;
    time_steps[299] = 0.01;
    time_steps[300] = 0.01;
    time_steps[301] = 0.01;
    time_steps[302] = 0.01;
    time_steps[303] = 0.01;
    time_steps[304] = 0.01;
    time_steps[305] = 0.01;
    time_steps[306] = 0.01;
    time_steps[307] = 0.01;
    time_steps[308] = 0.01;
    time_steps[309] = 0.01;
    time_steps[310] = 0.01;
    time_steps[311] = 0.01;
    time_steps[312] = 0.01;
    time_steps[313] = 0.01;
    time_steps[314] = 0.01;
    time_steps[315] = 0.01;
    time_steps[316] = 0.01;
    time_steps[317] = 0.01;
    time_steps[318] = 0.01;
    time_steps[319] = 0.01;
    time_steps[320] = 0.01;
    time_steps[321] = 0.01;
    time_steps[322] = 0.01;
    time_steps[323] = 0.01;
    time_steps[324] = 0.01;
    time_steps[325] = 0.01;
    time_steps[326] = 0.01;
    time_steps[327] = 0.01;
    time_steps[328] = 0.01;
    time_steps[329] = 0.01;
    time_steps[330] = 0.01;
    time_steps[331] = 0.01;
    time_steps[332] = 0.01;
    time_steps[333] = 0.01;
    time_steps[334] = 0.01;
    time_steps[335] = 0.01;
    time_steps[336] = 0.01;
    time_steps[337] = 0.01;
    time_steps[338] = 0.01;
    time_steps[339] = 0.01;
    time_steps[340] = 0.01;
    time_steps[341] = 0.01;
    time_steps[342] = 0.01;
    time_steps[343] = 0.01;
    time_steps[344] = 0.01;
    time_steps[345] = 0.01;
    time_steps[346] = 0.01;
    time_steps[347] = 0.01;
    time_steps[348] = 0.01;
    time_steps[349] = 0.01;
    time_steps[350] = 0.01;
    time_steps[351] = 0.01;
    time_steps[352] = 0.01;
    time_steps[353] = 0.01;
    time_steps[354] = 0.01;
    time_steps[355] = 0.01;
    time_steps[356] = 0.01;
    time_steps[357] = 0.01;
    time_steps[358] = 0.01;
    time_steps[359] = 0.01;
    time_steps[360] = 0.01;
    time_steps[361] = 0.01;
    time_steps[362] = 0.01;
    time_steps[363] = 0.01;
    time_steps[364] = 0.01;
    time_steps[365] = 0.01;
    time_steps[366] = 0.01;
    time_steps[367] = 0.01;
    time_steps[368] = 0.01;
    time_steps[369] = 0.01;
    time_steps[370] = 0.01;
    time_steps[371] = 0.01;
    time_steps[372] = 0.01;
    time_steps[373] = 0.01;
    time_steps[374] = 0.01;
    time_steps[375] = 0.01;
    time_steps[376] = 0.01;
    time_steps[377] = 0.01;
    time_steps[378] = 0.01;
    time_steps[379] = 0.01;
    time_steps[380] = 0.01;
    time_steps[381] = 0.01;
    time_steps[382] = 0.01;
    time_steps[383] = 0.01;
    time_steps[384] = 0.01;
    time_steps[385] = 0.01;
    time_steps[386] = 0.01;
    time_steps[387] = 0.01;
    time_steps[388] = 0.01;
    time_steps[389] = 0.01;
    time_steps[390] = 0.01;
    time_steps[391] = 0.01;
    time_steps[392] = 0.01;
    time_steps[393] = 0.01;
    time_steps[394] = 0.01;
    time_steps[395] = 0.01;
    time_steps[396] = 0.01;
    time_steps[397] = 0.01;
    time_steps[398] = 0.01;
    time_steps[399] = 0.01;

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
    W[1+(NY) * 0] = 0;
    W[1+(NY) * 1] = 2;
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
    W[2+(NY) * 0] = 0;
    W[2+(NY) * 1] = 0;
    W[2+(NY) * 2] = 2;
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
    W[3+(NY) * 0] = 0;
    W[3+(NY) * 1] = 0;
    W[3+(NY) * 2] = 0;
    W[3+(NY) * 3] = 30;
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
    W[5+(NY) * 0] = 0;
    W[5+(NY) * 1] = 0;
    W[5+(NY) * 2] = 0;
    W[5+(NY) * 3] = 0;
    W[5+(NY) * 4] = 0;
    W[5+(NY) * 5] = 0.1;
    W[5+(NY) * 6] = 0;
    W[5+(NY) * 7] = 0;
    W[5+(NY) * 8] = 0;
    W[5+(NY) * 9] = 0;
    W[5+(NY) * 10] = 0;
    W[5+(NY) * 11] = 0;
    W[5+(NY) * 12] = 0;
    W[5+(NY) * 13] = 0;
    W[6+(NY) * 0] = 0;
    W[6+(NY) * 1] = 0;
    W[6+(NY) * 2] = 0;
    W[6+(NY) * 3] = 0;
    W[6+(NY) * 4] = 0;
    W[6+(NY) * 5] = 0;
    W[6+(NY) * 6] = 0.5;
    W[6+(NY) * 7] = 0;
    W[6+(NY) * 8] = 0;
    W[6+(NY) * 9] = 0;
    W[6+(NY) * 10] = 0;
    W[6+(NY) * 11] = 0;
    W[6+(NY) * 12] = 0;
    W[6+(NY) * 13] = 0;
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
    W[10+(NY) * 10] = 0.001;
    W[10+(NY) * 11] = 0;
    W[10+(NY) * 12] = 0;
    W[10+(NY) * 13] = 0;
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
    W[11+(NY) * 11] = 0.001;
    W[11+(NY) * 12] = 0;
    W[11+(NY) * 13] = 0;
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
    W[12+(NY) * 12] = 0.001;
    W[12+(NY) * 13] = 0;
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
    W[13+(NY) * 13] = 0.001;

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
    Vu[12+(NY) * 0] = 1;
    Vu[12+(NY) * 1] = 0;
    Vu[13+(NY) * 0] = 0;
    Vu[13+(NY) * 1] = 1;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }






    double Zl[NS];
    double Zu[NS];
    double zl[NS];
    double zu[NS];
    
    Zl[0] = 0;
    Zl[1] = 0;
    Zl[2] = 0;
    Zl[3] = 0;
    Zl[4] = 0;
    Zl[5] = 0;
    Zl[6] = 0;
    Zl[7] = 0;

    
    Zu[0] = 0;
    Zu[1] = 0;
    Zu[2] = 0;
    Zu[3] = 0;
    Zu[4] = 0;
    Zu[5] = 0;
    Zu[6] = 0;
    Zu[7] = 0;

    
    zl[0] = 50;
    zl[1] = 50;
    zl[2] = 50;
    zl[3] = 50;
    zl[4] = 50;
    zl[5] = 50;
    zl[6] = 50;
    zl[7] = 50;

    
    zu[0] = 50;
    zu[1] = 50;
    zu[2] = 50;
    zu[3] = 50;
    zu[4] = 50;
    zu[5] = 50;
    zu[6] = 50;
    zu[7] = 50;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
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
    W_e[1+(NYN) * 0] = 0;
    W_e[1+(NYN) * 1] = 4;
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
    W_e[2+(NYN) * 0] = 0;
    W_e[2+(NYN) * 1] = 0;
    W_e[2+(NYN) * 2] = 4;
    W_e[2+(NYN) * 3] = 0;
    W_e[2+(NYN) * 4] = 0;
    W_e[2+(NYN) * 5] = 0;
    W_e[2+(NYN) * 6] = 0;
    W_e[2+(NYN) * 7] = 0;
    W_e[2+(NYN) * 8] = 0;
    W_e[2+(NYN) * 9] = 0;
    W_e[2+(NYN) * 10] = 0;
    W_e[2+(NYN) * 11] = 0;
    W_e[3+(NYN) * 0] = 0;
    W_e[3+(NYN) * 1] = 0;
    W_e[3+(NYN) * 2] = 0;
    W_e[3+(NYN) * 3] = 60;
    W_e[3+(NYN) * 4] = 0;
    W_e[3+(NYN) * 5] = 0;
    W_e[3+(NYN) * 6] = 0;
    W_e[3+(NYN) * 7] = 0;
    W_e[3+(NYN) * 8] = 0;
    W_e[3+(NYN) * 9] = 0;
    W_e[3+(NYN) * 10] = 0;
    W_e[3+(NYN) * 11] = 0;
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
    W_e[5+(NYN) * 0] = 0;
    W_e[5+(NYN) * 1] = 0;
    W_e[5+(NYN) * 2] = 0;
    W_e[5+(NYN) * 3] = 0;
    W_e[5+(NYN) * 4] = 0;
    W_e[5+(NYN) * 5] = 0.2;
    W_e[5+(NYN) * 6] = 0;
    W_e[5+(NYN) * 7] = 0;
    W_e[5+(NYN) * 8] = 0;
    W_e[5+(NYN) * 9] = 0;
    W_e[5+(NYN) * 10] = 0;
    W_e[5+(NYN) * 11] = 0;
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
    W_e[10+(NYN) * 10] = 0.002;
    W_e[10+(NYN) * 11] = 0;
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
    W_e[11+(NYN) * 11] = 0.002;
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
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);



    /**** Constraints ****/

    // bounds for initial stage

    // x0
    int idxbx0[12];
    
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

    double lbx0[12];
    double ubx0[12];
    
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
    lbx0[7] = 0;
    ubx0[7] = 0;
    lbx0[8] = 0;
    ubx0[8] = 0;
    lbx0[9] = 0;
    ubx0[9] = 0;
    lbx0[10] = 0;
    ubx0[10] = 0;
    lbx0[11] = 0;
    ubx0[11] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);


    // idxbxe_0
    int idxbxe_0[12];
    
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
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);


    /* constraints that are the same for initial and intermediate */



    // u
    int idxbu[NBU];
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    double lbu[NBU];
    double ubu[NBU];
    
    lbu[0] = -90;
    ubu[0] = 90;
    lbu[1] = -90;
    ubu[1] = 90;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }







    // set up soft bounds for nonlinear constraints
    int idxsh[NSH];
    
    idxsh[0] = 0;
    idxsh[1] = 1;
    idxsh[2] = 2;
    idxsh[3] = 3;
    idxsh[4] = 4;
    idxsh[5] = 5;
    idxsh[6] = 6;
    idxsh[7] = 7;
    double lsh[NSH];
    double ush[NSH];
    
    lsh[0] = -0.2;
    ush[0] = 0;
    lsh[1] = -0.2;
    ush[1] = 0;
    lsh[2] = -0.2;
    ush[2] = 0;
    lsh[3] = -0.2;
    ush[3] = 0;
    lsh[4] = -0.2;
    ush[4] = 0;
    lsh[5] = -0.2;
    ush[5] = 0;
    lsh[6] = -0.2;
    ush[6] = 0;
    lsh[7] = -0.2;
    ush[7] = 0;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ush", ush);
    }





    // x
    int idxbx[NBX];
    
    idxbx[0] = 3;
    idxbx[1] = 4;
    idxbx[2] = 5;
    idxbx[3] = 10;
    idxbx[4] = 11;
    double lbx[NBX];
    double ubx[NBX];
    
    lbx[0] = -1.5;
    ubx[0] = 1.5;
    lbx[1] = -1.5;
    ubx[1] = 1.5;
    lbx[2] = -5;
    ubx[2] = 5;
    lbx[3] = -30;
    ubx[3] = 36.5;
    lbx[4] = -30;
    ubx[4] = 36.5;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }





    // set up nonlinear constraints for stage 0 to N-1 
    double lh[NH];
    double uh[NH];

    
    lh[0] = 1.5;
    lh[1] = 1.5;
    lh[2] = 1.5;
    lh[3] = 1.5;
    lh[4] = 1.5;
    lh[5] = 1.5;
    lh[6] = 1.5;
    lh[7] = 1.5;

    
    uh[0] = 1000000;
    uh[1] = 1000000;
    uh[2] = 1000000;
    uh[3] = 1000000;
    uh[4] = 1000000;
    uh[5] = 1000000;
    uh[6] = 1000000;
    uh[7] = 1000000;
    
    for (int i = 0; i < N; i++)
    {
        // nonlinear constraints for stages 0 to N-1
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                     &nl_constr_h_fun_jac[i]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                    &nl_constr_h_fun[i]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
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
    double x0[12];

    // initialize with x0
    
    x0[0] = 0;
    x0[1] = 0;
    x0[2] = 1;
    x0[3] = 0.001;
    x0[4] = 0;
    x0[5] = 0;
    x0[6] = 0;
    x0[7] = 0;
    x0[8] = 0;
    x0[9] = 0;
    x0[10] = 0;
    x0[11] = 0;


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



    // initialize parameters to nominal value
    double p[16];
    
    p[0] = 100;
    p[1] = 100;
    p[2] = 100;
    p[3] = 100;
    p[4] = 100;
    p[5] = 100;
    p[6] = 100;
    p[7] = 100;
    p[8] = 100;
    p[9] = 100;
    p[10] = 100;
    p[11] = 100;
    p[12] = 100;
    p[13] = 100;
    p[14] = 100;
    p[15] = 100;


    for (int ii = 0; ii < N; ii++)
    {
        forw_vde_casadi[ii].set_param(forw_vde_casadi+ii, p);
        expl_ode_fun[ii].set_param(expl_ode_fun+ii, p);
    }


    // cost

    // constraints

    for (int ii = 0; ii < N; ii++)
    {
        nl_constr_h_fun_jac[ii].set_param(nl_constr_h_fun_jac+ii, p);
        nl_constr_h_fun[ii].set_param(nl_constr_h_fun+ii, p);
    }

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

    int casadi_np = 16;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    if (stage < 400)
    {
        forw_vde_casadi[stage].set_param(forw_vde_casadi+stage, p);
        expl_ode_fun[stage].set_param(expl_ode_fun+stage, p);
    

        // constraints
    
        nl_constr_h_fun_jac[stage].set_param(nl_constr_h_fun_jac+stage, p);
        nl_constr_h_fun[stage].set_param(nl_constr_h_fun+stage, p);

        // cost

    }
    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    
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
    for (int i = 0; i < 400; i++)
    {
        external_function_param_casadi_free(&forw_vde_casadi[i]);
        external_function_param_casadi_free(&expl_ode_fun[i]);
    }
    free(forw_vde_casadi);
    free(expl_ode_fun);

    // cost

    // constraints
    for (int i = 0; i < 400; i++)
    {
        external_function_param_casadi_free(&nl_constr_h_fun_jac[i]);
        external_function_param_casadi_free(&nl_constr_h_fun[i]);
    }
    free(nl_constr_h_fun_jac);
    free(nl_constr_h_fun);

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
