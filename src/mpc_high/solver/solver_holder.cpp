#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "solver_holder.h"

#include <stdio.h>
#include <math.h>
#include "ros/ros.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  17*/
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. 17 */
#define NOD         ACADO_NOD  /* Number of online data values. 59 */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. 23 */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. 6 */

#define N           ACADO_N   /* Number of intervals in the horizon. 10 */

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

// double * MPC_solver::solve_mpc(double input_arr[68], double cgoal[3]) {
	double * MPC_solver::solve_mpc(double input_arr[572], double cgoal[3]) {
	/* currentState_targetValue parsing:
	0 - 6: current theta
	6 - 12: goal theta
	12- 68: human spheres

	output j_dot[6]
	*/
	double x0[6] = {input_arr[0], input_arr[1], input_arr[2],input_arr[3], input_arr[4], input_arr[5]};
	double yN[6] = {input_arr[6], input_arr[7], input_arr[8], input_arr[9],input_arr[10], input_arr[11]};
	double human_spheres[590] = {input_arr[12], input_arr[13], input_arr[14], input_arr[15], 
							  input_arr[16], input_arr[17], input_arr[18], input_arr[19], 
							  input_arr[20], input_arr[21], input_arr[22], input_arr[23], 
							  input_arr[24], input_arr[25], input_arr[26], input_arr[27], 
							  input_arr[28], input_arr[29], input_arr[30], input_arr[31], 
							  input_arr[32], input_arr[33], input_arr[34], input_arr[35], 
						 	  input_arr[36], input_arr[37], input_arr[38], input_arr[39], 
							  input_arr[40], input_arr[41], input_arr[42], input_arr[43], 
							  input_arr[44], input_arr[45], input_arr[46], input_arr[47], 
							  input_arr[48], input_arr[49], input_arr[50], input_arr[51], 
							  input_arr[52], input_arr[53], input_arr[54], input_arr[55], 
							  input_arr[56], input_arr[57], input_arr[58], input_arr[59], 
							  input_arr[60], input_arr[61], input_arr[62], input_arr[63], 
							  input_arr[64], input_arr[65], input_arr[66], input_arr[67],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[68], input_arr[69], input_arr[70], input_arr[71], 
							  input_arr[72], input_arr[73], input_arr[74], input_arr[75], 
							  input_arr[76], input_arr[77], input_arr[78], input_arr[79], 
							  input_arr[80], input_arr[81], input_arr[82], input_arr[83], 
							  input_arr[84], input_arr[85], input_arr[86], input_arr[87], 
							  input_arr[88], input_arr[89], input_arr[90], input_arr[91], 
						 	  input_arr[92], input_arr[93], input_arr[94], input_arr[95], 
							  input_arr[96], input_arr[97], input_arr[98], input_arr[99], 
							  input_arr[100], input_arr[101], input_arr[102], input_arr[103], 
							  input_arr[104], input_arr[105], input_arr[106], input_arr[107], 
							  input_arr[108], input_arr[109], input_arr[110], input_arr[111], 
							  input_arr[112], input_arr[113], input_arr[114], input_arr[115], 
							  input_arr[116], input_arr[117], input_arr[118], input_arr[119], 
							  input_arr[120], input_arr[121], input_arr[122], input_arr[123],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[124], input_arr[125], input_arr[126], input_arr[127], 
							  input_arr[128], input_arr[129], input_arr[130], input_arr[131], 
							  input_arr[132], input_arr[133], input_arr[134], input_arr[135], 
							  input_arr[136], input_arr[137], input_arr[138], input_arr[139], 
							  input_arr[140], input_arr[141], input_arr[142], input_arr[143], 
							  input_arr[144], input_arr[145], input_arr[146], input_arr[147], 
						 	  input_arr[148], input_arr[149], input_arr[150], input_arr[151], 
							  input_arr[152], input_arr[153], input_arr[154], input_arr[155], 
							  input_arr[156], input_arr[157], input_arr[158], input_arr[159], 
							  input_arr[160], input_arr[161], input_arr[162], input_arr[163], 
							  input_arr[164], input_arr[165], input_arr[166], input_arr[167], 
							  input_arr[168], input_arr[169], input_arr[170], input_arr[171], 
							  input_arr[172], input_arr[173], input_arr[174], input_arr[175], 
							  input_arr[176], input_arr[177], input_arr[178], input_arr[179],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[180], input_arr[181], input_arr[182], input_arr[183], 
							  input_arr[184], input_arr[185], input_arr[186], input_arr[187], 
							  input_arr[188], input_arr[189], input_arr[190], input_arr[191], 
							  input_arr[192], input_arr[193], input_arr[194], input_arr[195], 
							  input_arr[196], input_arr[197], input_arr[198], input_arr[199], 
							  input_arr[200], input_arr[201], input_arr[202], input_arr[203], 
						 	  input_arr[204], input_arr[205], input_arr[206], input_arr[207], 
							  input_arr[208], input_arr[209], input_arr[210], input_arr[211], 
							  input_arr[212], input_arr[213], input_arr[214], input_arr[215], 
							  input_arr[216], input_arr[217], input_arr[218], input_arr[219], 
							  input_arr[220], input_arr[221], input_arr[222], input_arr[223], 
							  input_arr[224], input_arr[225], input_arr[226], input_arr[227], 
							  input_arr[228], input_arr[229], input_arr[230], input_arr[231], 
							  input_arr[232], input_arr[233], input_arr[234], input_arr[235],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[236], input_arr[237], input_arr[238], input_arr[239], 
							  input_arr[240], input_arr[241], input_arr[242], input_arr[243], 
							  input_arr[244], input_arr[245], input_arr[246], input_arr[247], 
							  input_arr[248], input_arr[249], input_arr[250], input_arr[251], 
							  input_arr[252], input_arr[253], input_arr[254], input_arr[255], 
							  input_arr[256], input_arr[257], input_arr[258], input_arr[259], 
						 	  input_arr[260], input_arr[261], input_arr[262], input_arr[263], 
							  input_arr[264], input_arr[265], input_arr[266], input_arr[267], 
							  input_arr[268], input_arr[269], input_arr[270], input_arr[271], 
							  input_arr[272], input_arr[273], input_arr[274], input_arr[275], 
							  input_arr[276], input_arr[277], input_arr[278], input_arr[279], 
							  input_arr[280], input_arr[281], input_arr[282], input_arr[283], 
							  input_arr[284], input_arr[285], input_arr[286], input_arr[287], 
							  input_arr[288], input_arr[289], input_arr[290], input_arr[291],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[292], input_arr[293], input_arr[294], input_arr[295], 
							  input_arr[296], input_arr[297], input_arr[298], input_arr[299], 
							  input_arr[300], input_arr[301], input_arr[302], input_arr[303], 
							  input_arr[304], input_arr[305], input_arr[306], input_arr[307], 
							  input_arr[308], input_arr[309], input_arr[310], input_arr[311], 
							  input_arr[312], input_arr[313], input_arr[314], input_arr[315], 
						 	  input_arr[316], input_arr[317], input_arr[318], input_arr[319], 
							  input_arr[320], input_arr[321], input_arr[322], input_arr[323], 
							  input_arr[324], input_arr[325], input_arr[326], input_arr[327], 
							  input_arr[328], input_arr[329], input_arr[330], input_arr[331], 
							  input_arr[332], input_arr[333], input_arr[334], input_arr[335], 
							  input_arr[336], input_arr[337], input_arr[338], input_arr[339], 
							  input_arr[340], input_arr[341], input_arr[342], input_arr[343], 
							  input_arr[344], input_arr[345], input_arr[346], input_arr[347],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[348], input_arr[349], input_arr[350], input_arr[351], 
							  input_arr[352], input_arr[353], input_arr[354], input_arr[355], 
							  input_arr[356], input_arr[357], input_arr[358], input_arr[359], 
							  input_arr[360], input_arr[361], input_arr[362], input_arr[363], 
							  input_arr[364], input_arr[365], input_arr[366], input_arr[367], 
							  input_arr[368], input_arr[369], input_arr[370], input_arr[371], 
						 	  input_arr[372], input_arr[373], input_arr[374], input_arr[375], 
							  input_arr[376], input_arr[377], input_arr[378], input_arr[379], 
							  input_arr[380], input_arr[381], input_arr[382], input_arr[383], 
							  input_arr[384], input_arr[385], input_arr[386], input_arr[387], 
							  input_arr[388], input_arr[389], input_arr[390], input_arr[391], 
							  input_arr[392], input_arr[393], input_arr[394], input_arr[395], 
							  input_arr[396], input_arr[397], input_arr[398], input_arr[399], 
							  input_arr[400], input_arr[401], input_arr[402], input_arr[403],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[404], input_arr[405], input_arr[406], input_arr[407], 
							  input_arr[408], input_arr[409], input_arr[410], input_arr[411], 
							  input_arr[412], input_arr[413], input_arr[414], input_arr[415], 
							  input_arr[416], input_arr[417], input_arr[418], input_arr[419], 
							  input_arr[420], input_arr[421], input_arr[422], input_arr[423], 
							  input_arr[424], input_arr[425], input_arr[426], input_arr[427], 
						 	  input_arr[428], input_arr[429], input_arr[430], input_arr[431], 
							  input_arr[432], input_arr[433], input_arr[434], input_arr[435], 
							  input_arr[436], input_arr[437], input_arr[438], input_arr[439], 
							  input_arr[440], input_arr[441], input_arr[442], input_arr[443], 
							  input_arr[444], input_arr[445], input_arr[446], input_arr[447], 
							  input_arr[448], input_arr[449], input_arr[450], input_arr[451], 
							  input_arr[452], input_arr[453], input_arr[454], input_arr[455], 
							  input_arr[456], input_arr[457], input_arr[458], input_arr[459],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[460], input_arr[461], input_arr[462], input_arr[463], 
							  input_arr[464], input_arr[465], input_arr[466], input_arr[467], 
							  input_arr[468], input_arr[469], input_arr[470], input_arr[471], 
						 	  input_arr[472], input_arr[473], input_arr[474], input_arr[475], 
							  input_arr[476], input_arr[477], input_arr[478], input_arr[479], 
							  input_arr[480], input_arr[481], input_arr[482], input_arr[483], 
							  input_arr[484], input_arr[485], input_arr[486], input_arr[487], 
							  input_arr[488], input_arr[489], input_arr[490], input_arr[491], 
							  input_arr[492], input_arr[493], input_arr[494], input_arr[495], 
							  input_arr[496], input_arr[497], input_arr[498], input_arr[499], 
							  input_arr[500], input_arr[501], input_arr[502], input_arr[503],
							  input_arr[504], input_arr[505], input_arr[506], input_arr[507], 
							  input_arr[508], input_arr[509], input_arr[510], input_arr[511], 
							  input_arr[512], input_arr[513], input_arr[514], input_arr[515],
							  cgoal[0], cgoal[1], cgoal[2],

							  input_arr[516], input_arr[517], input_arr[518], input_arr[519], 
							  input_arr[520], input_arr[521], input_arr[522], input_arr[523], 
							  input_arr[524], input_arr[525], input_arr[526], input_arr[527], 
						 	  input_arr[528], input_arr[529], input_arr[530], input_arr[531], 
							  input_arr[532], input_arr[533], input_arr[534], input_arr[535], 
							  input_arr[536], input_arr[537], input_arr[538], input_arr[539], 
							  input_arr[540], input_arr[541], input_arr[542], input_arr[543], 
							  input_arr[544], input_arr[545], input_arr[546], input_arr[547], 
							  input_arr[548], input_arr[549], input_arr[550], input_arr[551], 
							  input_arr[552], input_arr[553], input_arr[554], input_arr[555], 
							  input_arr[556], input_arr[557], input_arr[558], input_arr[559],
							  input_arr[560], input_arr[561], input_arr[562], input_arr[563],
							  input_arr[564], input_arr[565], input_arr[566], input_arr[567], 
							  input_arr[568], input_arr[569], input_arr[570], input_arr[571], 
							  cgoal[0], cgoal[1], cgoal[2]};
	
	// printf("\nSolver current: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", x0[0], x0[1], x0[2], x0[3], x0[4], x0[5]);
	// printf("\nSolver goal: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", yN[0], yN[1], yN[2], yN[3], yN[4], yN[5]);
	/* Some temporary variables. */
	int    i, j, iter;
	acado_timer t;
	
	// in model we have 6 theta and 6 theta_dot, we assign fixed reference with zero velocity
	for (j = 0; j < N; j++) {
		for (i = 0; i < 6; i++)  acadoVariables.y[j*NY + i ] = yN[ i ];	
	}

	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = yN[ i ];

	/* Initialize online data. */
	for (j = 0; j < N; ++j) {
		for (i = 0; i < NOD; ++i)  acadoVariables.od[ j*NOD+i ] = human_spheres[j*59+i];
		// printf("\n %i, Human: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", yN[0], yN[1], yN[2], yN[3], yN[4], yN[5]);
		// for (i = 0; i < NOD; ++i)  acadoVariables.od[ j*NOD+i ] = human_spheres[i];
	}
	// for (int j=0;j<10;j++){
	// 	for (int i=0; i<14; i++){
    //   	printf("human %i = %f, %f, %f, %f\n", j*14+i, human_spheres[59*j+4*i], human_spheres[59*j+4*i+1], human_spheres[59*j+4*i+2], human_spheres[59*j+4*i+3]);
    // }
	// }
	


    for (i = 0; i < (N + 1); ++i)  {
        for (j = 0; j < 6; j++) {
			acadoVariables.x[ i*NX + j ] = x0[ j ] + (yN[ j ] - x0[ j ])/N*i;
		}
	}

	/* MPC: initialize the current state feedback. */
	#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = x0[ i ];
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

	ROS_INFO("Time: %.3g ms; KKT = %.3e; ", 1e3 * te, KKT_val);

	static double joint_commands[15];
    joint_commands[14] = 0;  // infisibility marker
    for (i = 0; i < 2*NU; ++i) {
        if (acadoVariables.u[i] != acadoVariables.u[i]){
            acadoVariables.u[i] = 0;
            joint_commands[14] = 1;
        }
	    joint_commands[i] = acadoVariables.u[i];
		// printf("Solver: JC %i = %f\n", i, joint_commands[i]);
	}
	joint_commands[12] = 1e3 * te;
	joint_commands[13] = KKT_val;
	
	if (KKT_val > 0.01) {
		acado_initializeSolver();

		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

		for (i = 0; i < 12; ++i) {
	    joint_commands[i] = 0.0;
		// printf("Solver: JC %i = %f\n", i, joint_commands[i]);
		}
		/* Initialize the measurements/reference. */
		for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	} else {
		acado_shiftStates(2, 0, 0);
		acado_shiftControls( 0 );
	}
	return joint_commands;
}
