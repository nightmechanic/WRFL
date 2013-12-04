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
#include <math.h>

#include "WRFL_Kalman.h"



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



