/*
 * WRFL_Kalman.c
 *	Source file for Kalman filter routines
 *
 *	Created on: Dec 1, 2013
 *  Author: Ran Katz (Nightmechanic)
 *
 *	The single dimension functions Kalman_Init and Kalman_Update
 *	are based on the tutorial from: http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx
 *
 *	The multi-dimension Kalman functions MD_Kalman_Init and MD_Kalman_Update are based
 *	on the pykalman python library in: https://github.com/pykalman/pykalman,
 *	and specifically on the standard Kalman filter implementation in the standard.py  file
 *
 *	The pykalman library is released under the following license:
 *	New BSD License
 *	Copyright (c) 2012 Daniel Duckworth.
 *	All rights reserved.
 *
 * (I would like to thank Daniel Duckworth for his great Kalman library that helped better understand
 * the Kalman filter, was used to simulate the apogee detection for teh WRFL project and is the basis
 * for this MD_Kalman implementation)
 *
 * This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/deed.en_US.
 */

#include <stdbool.h>
#include <stdint.h>
//#include <math.h>
#include "arm_math.h"

#include "WRFL_Kalman.h"
#ifdef CHECK_MATRIX_DIM
	#include "Nokia5110.h"
#endif

//temp variables
arm_matrix_instance_f32 mfPredictedState;
float32_t fPredictedState[DIM_STATE];

arm_matrix_instance_f32 mfPredictedStateCovariance;
float32_t fPredictedStateCovariance[DIM_STATE * DIM_STATE];

arm_matrix_instance_f32 mfPredictedObservationMean;
float32_t fPredictedObservationMean[DIM_OBS];

arm_matrix_instance_f32 mfPredictedObservationCovariance;
float32_t fPredictedObservationCovariance[DIM_OBS*DIM_OBS];

arm_matrix_instance_f32 mfTemp1_DimStateState, mfTemp2_DimStateState, mfTemp3_DimStateObs, mfTemp4_DimObsObs, mfTemp5_DimState1, mfTemp6_DimObsState;
float32_t fTemp1[DIM_STATE * DIM_STATE];
float32_t fTemp2[DIM_STATE * DIM_STATE];
float32_t fTemp3[DIM_STATE * DIM_OBS];
float32_t fTemp4[DIM_OBS * DIM_OBS];

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
									float *pfObservationMatrix_T,
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
	arm_mat_init_f32(&pKState->mfObservationMatrix , DIM_OBS, DIM_STATE, (float32_t *)pfObservationMatrix);
	arm_mat_init_f32(&pKState->mfObservationMatrix_T , DIM_STATE, DIM_OBS, (float32_t *)pfObservationMatrix_T);
	arm_mat_init_f32(&pKState->mfObservationCovariance , DIM_OBS, DIM_OBS, (float32_t *)pfObservationCovariance);
	arm_mat_init_f32(&pKState->mfGain , DIM_STATE, DIM_OBS, (float32_t *)pfGain);

	//init temp variables here to save time during runtime

	arm_mat_init_f32(&mfPredictedState, DIM_STATE, 1, (float32_t *)fPredictedState);
	arm_mat_init_f32(&mfPredictedStateCovariance, DIM_STATE, DIM_STATE, (float32_t *)fPredictedStateCovariance);
	arm_mat_init_f32(&mfPredictedObservationMean, DIM_OBS, 1, (float32_t *)fPredictedObservationMean);
	arm_mat_init_f32(&mfPredictedObservationCovariance, DIM_OBS, DIM_OBS, (float32_t *)fPredictedObservationCovariance);


	arm_mat_init_f32(&mfTemp1_DimStateState, DIM_STATE, DIM_STATE, (float32_t *)fTemp1);
	arm_mat_init_f32(&mfTemp2_DimStateState, DIM_STATE, DIM_STATE, (float32_t *)fTemp2);
	arm_mat_init_f32(&mfTemp3_DimStateObs, DIM_STATE, DIM_OBS, (float32_t *)fTemp3);
	arm_mat_init_f32(&mfTemp4_DimObsObs, DIM_OBS, DIM_OBS, (float32_t *)fTemp4);
	arm_mat_init_f32(&mfTemp5_DimState1, DIM_STATE,1, (float32_t *)fTemp1);
	arm_mat_init_f32(&mfTemp6_DimObsState, DIM_OBS, DIM_STATE, (float32_t *)fTemp3);

	//pre calculate the transposed transition matrix to save time during updates
	status = arm_mat_trans_f32(&pKState->mfTransitionMatrix, &pKState->mfTransitionMatrix_T);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K107");
	}
#endif
	status = arm_mat_trans_f32(&pKState->mfObservationMatrix, &pKState->mfObservationMatrix_T);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K117");
	}
#endif


}

void MD_Kalman_Update_Meas(tMD_Kalman_State *pKState, arm_matrix_instance_f32 mfmeasurement)
{
	arm_status status;

	//Time update (prediction)

	// predicted state update
	/*predicted_state_mean = (
	        np.dot(transition_matrix, current_state_mean)
	        + transition_offset
	    )*/

	status = arm_mat_mult_f32(&pKState->mfTransitionMatrix, &pKState->mfState, &mfPredictedState);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K140");
	}
#endif
	//predicted state covariance
	/*predicted_state_covariance = (
	        np.dot(transition_matrix,
	               np.dot(current_state_covariance,
	                      transition_matrix.T))
	        + transition_covariance
	    )*/

	status = arm_mat_mult_f32(&pKState->mfStateCovariance, &pKState->mfTransitionMatrix_T, &mfTemp1_DimStateState);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K155");
	}
#endif
	status = arm_mat_mult_f32(&pKState->mfTransitionMatrix,&mfTemp1_DimStateState, &mfTemp2_DimStateState);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K162");
	}
#endif
	status = arm_mat_add_f32(&mfTemp2_DimStateState , &pKState->mfTransitionCovariance , &mfPredictedStateCovariance);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K169");
	}
#endif



	//Measurement Update (correction)

	/*predicted_observation_mean = (
	            np.dot(observation_matrix,
	                   predicted_state_mean)
	            + observation_offset
	        )*/


	status = arm_mat_mult_f32(&pKState->mfObservationMatrix,&mfPredictedState,&mfPredictedObservationMean);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K188");
	}
#endif
	       /* predicted_observation_covariance = (
	            np.dot(observation_matrix,
	                   np.dot(predicted_state_covariance,
	                          observation_matrix.T))
	            + observation_covariance
	        )*/
	status = arm_mat_mult_f32(&mfPredictedStateCovariance,&pKState->mfObservationMatrix_T,&mfTemp3_DimStateObs);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K201");
	}
#endif
	status = arm_mat_mult_f32(&pKState->mfObservationMatrix,&mfTemp3_DimStateObs,&mfPredictedObservationCovariance);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K208");
	}
#endif
	status = arm_mat_add_f32(&mfPredictedObservationCovariance,&pKState->mfObservationCovariance,&mfPredictedObservationCovariance);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K215");
	}
#endif

	        /*kalman_gain = (
	            np.dot(predicted_state_covariance,
	                   np.dot(observation_matrix.T,
	                          linalg.pinv(predicted_observation_covariance)))
	        )*/
	status = arm_mat_inverse_f32(&mfPredictedObservationCovariance, &mfTemp4_DimObsObs);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K228");
	}
#endif
	status = arm_mat_mult_f32(&pKState->mfObservationMatrix_T, &mfTemp4_DimObsObs, &mfTemp3_DimStateObs);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K235");
	}
#endif
	status = arm_mat_mult_f32(&mfPredictedStateCovariance, &mfTemp3_DimStateObs, &pKState->mfGain);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K242");
	}
#endif

	        /*corrected_state_mean = (
	            predicted_state_mean
	            + np.dot(kalman_gain, observation - predicted_observation_mean)
	        )*/
	status = arm_mat_sub_f32(&mfmeasurement, &mfPredictedObservationMean, &mfPredictedObservationMean);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K254");
	}
#endif
	status = arm_mat_mult_f32(&pKState->mfGain, &mfPredictedObservationMean, &mfTemp5_DimState1);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K261");
	}
#endif
	status = arm_mat_add_f32(&mfPredictedState, &mfTemp5_DimState1, &pKState->mfState);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K268");
	}
#endif

	        /*corrected_state_covariance = (
	            predicted_state_covariance
	            - np.dot(kalman_gain,
	                     np.dot(observation_matrix,
	                            predicted_state_covariance))
	        )*/

	status = arm_mat_mult_f32(&pKState->mfObservationMatrix, &mfPredictedStateCovariance, &mfTemp6_DimObsState);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K283");
	}
#endif
	status =  arm_mat_mult_f32(&pKState->mfGain, &mfTemp6_DimObsState, &mfTemp1_DimStateState);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K290");
	}
#endif
	status = arm_mat_sub_f32(&mfPredictedStateCovariance, &mfTemp1_DimStateState, &pKState->mfStateCovariance);
#ifdef CHECK_MATRIX_DIM
	if (status != ARM_MATH_SUCCESS){
		 Nokia5110_SetCursor(0, 0);
		   Nokia5110_OutString("ERR_K297");
	}
#endif
}

