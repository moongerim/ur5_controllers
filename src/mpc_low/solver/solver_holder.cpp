#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "solver_holder.h"

#include <stdio.h>
#include <math.h>
#include "ros/ros.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

//#define NUM_STEPS   5        /* Number of real-time iterations. */
//#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

//MPC_solver::MPC_solver() : num_steps(0) { }
MPC_solver::MPC_solver(int n): num_steps(n) {
	int i;
	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0;
};

int MPC_solver::reinitialize(){
	int i;
	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0;
	return 0;
};

double * MPC_solver::solve_mpc(double input_arr[71], double state_arr[60]) {
	/* currentState_targetValue parsing:
	0 - 6: current theta
	6 - 12: goal theta
	12 - 68: human spheres
	output j_dot[7]
	*/
	
	double x0[6] = {input_arr[0], input_arr[1], input_arr[2], input_arr[3], input_arr[4], input_arr[5]};
	double yN[6] = {input_arr[6], input_arr[7], input_arr[8], input_arr[9], input_arr[10], input_arr[11]};
	double human_spheres[59] = {input_arr[12], input_arr[13], input_arr[14], input_arr[15], input_arr[16], 
							  input_arr[17], input_arr[18], input_arr[19], input_arr[20], input_arr[21],
							  input_arr[22], input_arr[23], input_arr[24], input_arr[25], input_arr[26], 
							  input_arr[27], input_arr[28], input_arr[29], input_arr[30], input_arr[31], 
							  input_arr[32], input_arr[33], input_arr[34], input_arr[35], input_arr[36], 
							  input_arr[37], input_arr[38], input_arr[39], input_arr[40], input_arr[41],
							  input_arr[42], input_arr[43], input_arr[44], input_arr[45], input_arr[46], 
							  input_arr[47], input_arr[48], input_arr[49], input_arr[50], input_arr[51], 
							  input_arr[52], input_arr[53], input_arr[54], input_arr[55], input_arr[56], 
							  input_arr[57], input_arr[58], input_arr[59], input_arr[60], input_arr[61],
							  input_arr[62], input_arr[63], input_arr[64], input_arr[65], input_arr[66], 
							  input_arr[67], input_arr[68], input_arr[69], input_arr[70]};
	
	// printf("\ncurrent: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6] );
	// printf("\n___goal: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", yN[0], yN[1], yN[2], yN[3], yN[4], yN[5], yN[6] );
	/* Some temporary variables. */
	
	int    i, j, iter;
	acado_timer t;
	
	// in model we have 7 theta and 7 theta_dot, we assign fixed reference with zero velocity
	// tracking goal
	for (j = 0; j < N; j++) {
		//for (i = 0; i < 7; i++)  acadoVariables.y[j*NY + i ] = yN[ i ];
        for (i = 0; i < 6; i++)  acadoVariables.y[j*NY+i] = state_arr[j*6+i];	
	}
	
        // terminal goal
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = yN[i];

	/* Initialize online data. */
	for (j = 0; j < N; ++j) {
		for (i = 0; i < NOD; ++i)  acadoVariables.od[j*NOD+i] = human_spheres[i];
	}
        //printf("values %.3f %.3f %.3f %.3f", human_spheres[44], human_spheres[45], human_spheres[46], human_spheres[47]);

	/* MPC: initialize the current state feedback. */
	#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < 6; ++i) acadoVariables.x0[ i ] = x0[i];
	#endif
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;
	acado_tic( &t );

	for(iter = 0; iter < num_steps; ++iter)
	{
		/* Prepare for the RTI step. */
		acado_preparationStep();
		/* Compute the feedback step. */
		acado_feedbackStep( );
	}

	real_t te = acado_toc( &t );
	real_t KKT_val = acado_getKKT();

	ROS_INFO("Time: %.3g ms; KKT = %.3e", 1e3 * te, KKT_val);

	static double joint_commands[9];
    joint_commands[8] = 0;  // infisibility marker
    for (i = 0; i < 6; ++i) {
        if (acadoVariables.u[i] != acadoVariables.u[i]){
            acadoVariables.u[i] = 0;
            joint_commands[9] = 1;
        }
	    joint_commands[i] = acadoVariables.u[i];
	}
	joint_commands[6] = 1e3 * te;
	joint_commands[7] = KKT_val;

	if (KKT_val > 0.01) {
		acado_initializeSolver();

		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0;
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.0;

		/* Initialize the measurements/reference. */
		for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0;
	} else {
		acado_shiftStates(2, 0, 0);
		acado_shiftControls( 0 );
	}
	
	return joint_commands;
}
