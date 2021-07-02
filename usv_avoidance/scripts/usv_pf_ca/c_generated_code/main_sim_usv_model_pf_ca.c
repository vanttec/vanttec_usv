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
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_usv_model_pf_ca.h"


int main()
{
    int status = 0;
    status = usv_model_pf_ca_acados_sim_create();

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    // initial condition
    double x_current[12];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 1;
    x_current[3] = 0.001;
    x_current[4] = 0;
    x_current[5] = 0;
    x_current[6] = 0;
    x_current[7] = 0;
    x_current[8] = 0;
    x_current[9] = 0;
    x_current[10] = 0;
    x_current[11] = 0;
    
  


    // initial value for control input
    double u0[2];
    u0[0] = 0.0;
    u0[1] = 0.0;
    // set parameters
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
    

    usv_model_pf_ca_acados_sim_update_params(p, 16);
  

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(usv_model_pf_ca_sim_config, usv_model_pf_ca_sim_dims,
            usv_model_pf_ca_sim_in, "x", x_current);
        status = usv_model_pf_ca_acados_sim_solve();

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(usv_model_pf_ca_sim_config, usv_model_pf_ca_sim_dims,
               usv_model_pf_ca_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < 12; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = usv_model_pf_ca_acados_sim_free();
    if (status) {
        printf("usv_model_pf_ca_acados_sim_free() returned status %d. \n", status);
    }

    return status;
}
