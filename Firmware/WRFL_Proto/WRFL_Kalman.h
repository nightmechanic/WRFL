/*
 * WRFL_Kalman.h
 * Header file for Kalman filter routines
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


#include "arm_math.h"

/////////////// Single Dimension Kalman filter ////////////////

typedef struct {
	float X;
	float P;
	float K;
	float R;
	float Q;

} tKalman_State;

extern void Kalman_Init(tKalman_State *pKState, float fX_init, float fP_init, float fR_init, float fQ_init);
extern void Kalman_Update(tKalman_State *pKState, float fmeasurement);

/////////////// Multi Dimension Kalman filter ////////////////

#define DIM_STATE	3
#define DIM_OBS		1


typedef struct {
	arm_matrix_instance_f32 mfState; //  X
	arm_matrix_instance_f32 mfStateCovariance;  // P
	arm_matrix_instance_f32 mfTransitionMatrix; // A
	arm_matrix_instance_f32 mfTransitionMatrix_T; // A transposed
	arm_matrix_instance_f32 mfTransitionCovariance; //Q
	arm_matrix_instance_f32 mfObservationMatrix; // H or C
	arm_matrix_instance_f32 mfObservationMatrix_T; // H or C transposed
	arm_matrix_instance_f32 mfObservationCovariance; //R
	arm_matrix_instance_f32 mfGain; //K

} tMD_Kalman_State;

extern void MD_Kalman_Init(tMD_Kalman_State *pKState,
									float *pfState,
									float *pfStateCovariance,
									float *pfTransitionMatrix,
									float *pfTransitionMatrix_T,
									float *pfTransitionCovariance,
									float *pfObservationMatrix,
									float *pfObservationMatrix_T,
									float *pfObservationCovariance,
									float *pfGain);

extern void MD_Kalman_Update_Meas(tMD_Kalman_State *pKState, arm_matrix_instance_f32 fmeasurement);
