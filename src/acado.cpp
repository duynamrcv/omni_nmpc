// #include "../acado_mpc_export/acado_common.h"
// #include "../acado_mpc_export/acado_auxiliary_functions.h"
#include "acado.h"

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using namespace std;

void init_weight()
{
	for (int i = 0; i < N; i++)
	{
		// Setup diagonal entries
		acadoVariables.W[NY * NY * i + (NY + 1) * 0] = 1; //weight_x;
		acadoVariables.W[NY * NY * i + (NY + 1) * 1] = 1; //weight_y;
		acadoVariables.W[NY * NY * i + (NY + 1) * 2] = 1; //weight_q;
		acadoVariables.W[NY * NY * i + (NY + 1) * 3] = 1; //weight_vx;
        acadoVariables.W[NY * NY * i + (NY + 1) * 4] = 1; //weight_vy;
		acadoVariables.W[NY * NY * i + (NY + 1) * 5] = 1; //weight_w;
	}
	acadoVariables.WN[(NYN + 1) * 0] = 1; //weight_x;
	acadoVariables.WN[(NYN + 1) * 1] = 1; //weight_y;
	acadoVariables.WN[(NYN + 1) * 2] = 1; //weight_q;
}

vector<vector<double>> init_acado()
{
    /* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (int i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (int i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (int i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

    acado_preparationStep();
	vector<double> control_output_vx;
	vector<double> control_output_vy;
    vector<double> control_output_w;
	for (int i = 0; i < ACADO_N; ++i)
	{
        // There are 3 outputs vx, vy, w
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_vx.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
            else if(j == 1 )
            {
                control_output_vy.push_back(acadoVariables.u[i * ACADO_NU + j]);
            }
			else 
			{
				control_output_w.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
	init_weight();
	return {control_output_vx, control_output_vy, control_output_w};
}

vector<vector<double>> run_mpc_acado(vector<double> states,
                                    vector<double> ref_states,
                                    vector<vector<double>> previous_u)
{
    /* Some temporary variables. */
	int i, iter;
	acado_timer t;

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  
	{
		acadoVariables.x[ i ] = (real_t) states[i];
	}
    for (i = 0; i < NX; ++i)
	{
		acadoVariables.x0[i] = (real_t)states[i];
	}

    /* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)
	{
		acadoVariables.y[i] = (real_t)ref_states[i];
	}
	for (i = 0; i < NYN; ++i)
	{
		acadoVariables.yN[i] = (real_t)ref_states[NY * (N - 1) + i];
	}

	// /* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic(&t);

	/* The "real-time iterations" loop. */

	for (iter = 0; i < NUM_STEPS; ++i)
	{
		/* Perform the feedback step. */
		acado_feedbackStep();
		acado_preparationStep();

		/* Optional: shift the initialization (look at acado_common.h). */
		/* acado_shiftStates(2, 0, 0); */
		/* acado_shiftControls( 0 ); */
	}


	// // Reference
	// for (int i = 0; i < N; i++)
	// {
	// 	cout << "Reference " << i << " "
	// 			<< acadoVariables.y[NY*i + 0] << " "
	// 			<< acadoVariables.y[NY*i + 1] << " "
	// 			<< acadoVariables.y[NY*i + 2] << " "
	// 			<< acadoVariables.y[NY*i + 3] << " "
	// 			<< acadoVariables.y[NY*i + 4] << " "
	// 			<< acadoVariables.y[NY*i + 5] << endl;
	// }

	// for (int i = 0; i < N; i++)
	// {
	// 	cout << "Control " << i << " "
	// 			<< acadoVariables.u[NU * i + 0] << " "
	// 			<< acadoVariables.u[NU * i + 1] << " "
	// 			<< acadoVariables.u[NU * i + 2] << endl;
	// }

	// // state:
	// for (int i = 0; i < N; i++)
	// {
	// 	cout << "State " << i << " "
	// 			<< acadoVariables.x[NX * i + 0] << " "
	// 			<< acadoVariables.x[NX * i + 1] << " "
	// 			<< acadoVariables.x[NX * i + 2] << endl;
	// }

	/* Read the elapsed time. */
	real_t te = acado_toc(&t);

    vector<double> control_output_vx;
	vector<double> control_output_vy;
    vector<double> control_output_w;
	real_t *u = acado_getVariablesU();
	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_vx.push_back((double)u[i * ACADO_NU + j]);
			}
            else if (j == 1)
            {
                control_output_vy.push_back((double)u[i * ACADO_NU + j]);
            }
			else
			{
				control_output_w.push_back((double)u[i * ACADO_NU + j]);
			}
		}
	}
	// cout << control_output_vx[0] << " " << control_output_vy[0] << " " << control_output_w[0] << endl;
	return {control_output_vx, control_output_vy, control_output_w};
}

vector<double> calculate_ref_states(const vector<double> &ref_x,
									const vector<double> &ref_y,
									const vector<double> &ref_q,
                                    const double &reference_vx,
                                    const double &reference_vy,
                                    const double &reference_w)
{
	vector<double> result;
	for (int i = 0; i < N; i++)
	{
		result.push_back(ref_x[i]);
		result.push_back(ref_y[i]);
		result.push_back(ref_q[i]);
		result.push_back(0);
		result.push_back(0);
		result.push_back(0);
	}
	return result;
}

vector<double> update_states(vector<double> state, double vx_cmd, double vy_cmd, double w_cmd)
{
	// based on kinematic model
	double x0 = state[0];
	double y0 = state[1];
	double q0 = state[2];
	double vx0 = vx_cmd;
	double vy0 = vy_cmd;
	double w0 = w_cmd;

	double x1 = x0 + (vx0 * cos(q0) - vy0 * sin(q0))* Ts;
	double y1 = y0 + (vx0 * sin(q0) + vy0 * cos(q0))* Ts;
	double q1 = q0 + w0 * Ts;
	return {x1, y1, q1};
}

vector<double> motion_prediction(const vector<double> &cur_states,
                                const vector<vector<double>> &prev_u)
{
	vector<double> old_vx_cmd = prev_u[0];
	vector<double> old_vy_cmd = prev_u[1];
	vector<double> old_w_cmd = prev_u[2];

	vector<vector<double>> predicted_states;
	predicted_states.push_back(cur_states);

	for (int i = 0; i < N; i++)
	{
		vector<double> cur_state = predicted_states[i];
		// yaw angle compensation of overflow
		if (cur_state[3] > M_PI)
		{
			cur_state[3] -= 2 * M_PI;
		}
		if (cur_state[3] < -M_PI)
		{
			cur_state[3] += 2 * M_PI;
		}
		vector<double> next_state = update_states(cur_state, old_vx_cmd[i], old_vy_cmd[i], old_w_cmd[i]);
		predicted_states.push_back(next_state);
	}

	vector<double> result;
	for (int i = 0; i < (ACADO_N + 1); ++i)
	{
		for (int j = 0; j < NX; ++j)
		{
			result.push_back(predicted_states[i][j]);
		}
	}
	return result;
}