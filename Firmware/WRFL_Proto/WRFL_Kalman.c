/*
 * WRFL_Kalman.h
 *	Source file for Kalman filter routines
 *	this is for a single dimension, no control Kalman filter
 *	Based on the tutorial from: http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx
 *
 *  Created on: Dec 1, 2013
 *  Author: Ran Katz (Nightmechanic)
 *
 * This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/deed.en_US.
 */

#include <stdbool.h>
#include <stdint.h>
//#include <math.h>
#include "arm_math.h"

#include "WRFL_Kalman.h"

//temp variables
arm_matrix_instance_f32 mfPredictedState;
float32_t fPredictedState[DIM_STATE];

arm_matrix_instance_f32 mfPredictedStateCovariance;
float32_t fPredictedStateCovariance[DIM_STATE * DIM_STATE];

arm_matrix_instance_f32 mfTempDimStateState_1, mfTempDimStateState_2;
float32_t fTemp1[DIM_STATE * DIM_STATE];
float32_t fTemp2[DIM_STATE * DIM_STATE];

/////////////// Single Dimension Kalman filter ////////////////

void Kalman_Init(tKalman_State *pKState, float fX_init, float fP_init, float fR_init, float fQ_init)
{
	pKState->X = fX_init;
	pKState->P = fP_init;
	pKState->R = fR_init;
	pKState->Q = fQ_init;
}


void Kalman_Update(tKalman_State *pKState, float fmeasurement)
{
	//Time update (prediction)
	//no need to update X
	pKState->P = pKState->P + pKState->Q;

	//Measurement Update (correction)
	pKState->K = (pKState->P)/(pKState->P + pKState->R);
	pKState->X = pKState->X + (pKState->K)*(fmeasurement - pKState->X);
	pKState->P = (1.0f - pKState->K) * (pKState->P);
}


/////////////// Multi Dimension Kalman filter ////////////////

void MD_Kalman_Init(tMD_Kalman_State *pKState,
									float *pfState,
									float *pfStateCovariance,
									float *pfTransitionMatrix,
									float *pfTransitionMatrix_T,
									float *pfTransitionCovariance,
									float *pfObservationMatrix,
									float *pfObservationCovariance,
									float *pfGain)
{
	arm_status status;

	// initialize MD_Kalman matrices

	arm_mat_init_f32(&pKState->mfState, DIM_STATE, 1, (float32_t *)pfState);
	arm_mat_init_f32(&pKState->mfStateCovariance, DIM_STATE, DIM_STATE, (float32_t *)pfStateCovariance);
	arm_mat_init_f32(&pKState->mfTransitionMatrix, DIM_STATE, DIM_STATE, (float32_t *)pfTransitionMatrix);
	arm_mat_init_f32(&pKState->mfTransitionMatrix_T, DIM_STATE, DIM_STATE, (float32_t *)pfTransitionMatrix_T);
	arm_mat_init_f32(&pKState->mfTransitionCovariance , DIM_STATE, DIM_STATE, (float32_t *)pfTransitionCovariance);
	arm_mat_init_f32(&pKState->mfObservationMatrix , DIM_OBS, 1, (float32_t *)pfObservationMatrix);
	arm_mat_init_f32(&pKState->mfObservationCovariance , DIM_OBS, DIM_STATE, (float32_t *)pfObservationCovariance);
	arm_mat_init_f32(&pKState->mfGain , DIM_STATE, 1, (float32_t *)pfGain);

	//init temp variables

	arm_mat_init_f32(&mfPredictedState, DIM_STATE, 1, (float32_t *)fPredictedState);
	arm_mat_init_f32(&mfPredictedStateCovariance, DIM_STATE, DIM_STATE, (float32_t *)fPredictedStateCovariance);

	arm_mat_init_f32(&mfTempDimStateState_1, DIM_STATE, DIM_STATE, (float32_t *)fTemp1);
	arm_mat_init_f32(&mfTempDimStateState_2, DIM_STATE, DIM_STATE, (float32_t *)fTemp2);

	//pre calculate the transposed transition matrix to save time during updates
	status = arm_mat_trans_f32(&pKState->mfTransitionMatrix, &pKState->mfTransitionMatrix_T);



}

void MD_Kalman_Update_Meas(tMD_Kalman_State *pKState, arm_matrix_instance_f32 fmeasurement)
{
	arm_status status;

	//Time update (prediction)

	// predicted state update
	status = arm_mat_mult_f32(&pKState->mfTransitionMatrix, &pKState->mfState, &mfPredictedState);

	//predicted state covariance

	status = arm_mat_mult_f32(&pKState->mfStateCovariance, &pKState->mfTransitionMatrix_T, &mfTempDimStateState_1);

	status = arm_mat_mult_f32(&pKState->mfTransitionMatrix,&mfTempDimStateState_1, &mfTempDimStateState_2);

	status = arm_mat_add_f32(&mfTempDimStateState_2 , &pKState->mfTransitionCovariance , &mfPredictedStateCovariance);




	//Measurement Update (correction)

}

