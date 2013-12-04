/*
 * WRFL_Kalman.h
 *	Header file for Kalman filter routines
 *	this is for a single dimension, no control Kalman filter
 *	Based on the tutorial from: http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx
 *
 *  Created on: Dec 1, 2013
 *  Author: Ran Katz (Nightmechanic)
 *
 * This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/deed.en_US.
 */

#ifndef KALMAN_ALTIUNO_H_
#define KALMAN_ALTIUNO_H_


typedef struct {
	float X;
	float P;
	float K;
	float R;
	float Q;

} tKalman_State;

extern void Kalman_Init(tKalman_State *pKState, float fX_init, float fP_init, float fR_init, float fQ_init);
extern void Kalman_Update(tKalman_State *pKState, float fmeasurement);


#endif /* KALMAN_ALTIUNO_H_ */
