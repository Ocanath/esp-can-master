/*
 * sauron-eye-closedform-ik.c
 *
 *  Created on: Apr 11, 2025
 *      Author: ocanath
 */
#include "sin-math.h"

const float BasePlaneDistance = 35.f/2.f+19.84f;

//todo implement or use std math
float arcsin(float)
{
	return 0;
}
//todo implement or use std math
float sqrtf(float)
{
	return 0;
}

int get_ik_angles_float(float vx, float vy, float vz, float * theta1 , float*theta2)
{
	float vx_pow2 = vx*vx;
	float vx_pow4 = vx_pow2*vx_pow2;
	float vy_pow2 = vy*vy;
	float vz_pow2 = vz*vz;
	float vz_pow4 = vz_pow2*vz_pow2;


	float sqrt_operand = vx_pow4 + vx_pow2*vy_pow2 + 2*vx_pow2*vz_pow2 + vy_pow2*vz_pow2 + vz_pow4;
	if(sqrt_operand < 0)
		return -1;
	float O2Targy_0 = -BasePlaneDistance*vy*vz/sqrtf(sqrt_operand);
	sqrt_operand = vx_pow4 + vx_pow2*vy_pow2 + 2*vx_pow2*vz_pow2 + vy_pow2*vz_pow2 + vz_pow4;
	float O2Targz_0 = BasePlaneDistance*vx*vy/sqrtf(sqrt_operand) + BasePlaneDistance;

	//todo: pre-compute arcsin operand and check for -1 to 1 bounds
	float theta2_s2 = -arcsin((BasePlaneDistance - O2Targz_0)/BasePlaneDistance);
	float theta1_s2 = -arcsin(O2Targy_0/(BasePlaneDistance*cos_fast(theta2_s2)));


	*theta1 = atan2_approx(vx, vz);
	*theta2 = theta1_s2;
	return 0;
}
