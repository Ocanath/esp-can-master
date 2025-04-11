/*
 * sauron-eye-closedform-ik.c
 *
 *  Created on: Apr 11, 2025
 *      Author: ocanath
 */
#include "sin-math.h"
#include <math.h>

const float BasePlaneDistance = 35.f/2.f+19.84f;



int get_ik_angles_float(float vx, float vy, float vz, float * theta1 , float*theta2)
{
	float vx_pow2 = vx*vx;
	float vx_pow4 = vx_pow2*vx_pow2;
	float vy_pow2 = vy*vy;
	float vz_pow2 = vz*vz;
	float vz_pow4 = vz_pow2*vz_pow2;


	float operand = vx_pow4 + vx_pow2*vy_pow2 + 2*vx_pow2*vz_pow2 + vy_pow2*vz_pow2 + vz_pow4;
	if(operand < 0)
	{
		return -1;
	}
	float O2Targy_0 = -BasePlaneDistance*vy*vz/ sqrtf(operand);
	operand = vx_pow4 + vx_pow2*vy_pow2 + 2*vx_pow2*vz_pow2 + vy_pow2*vz_pow2 + vz_pow4;
	if(operand < 0)
	{
		return -1;
	}
	float O2Targz_0 = BasePlaneDistance*vx*vy/ sqrtf(operand) + BasePlaneDistance;

	//todo: pre-compute arcsin operand and check for -1 to 1 bounds
	operand = (BasePlaneDistance - O2Targz_0)/BasePlaneDistance;
	if(operand < -1 || operand > 1)
	{
		return -1;
	}
	float theta2_s2 = -asinf(operand);

	operand = O2Targy_0/(BasePlaneDistance*cosf(theta2_s2));
	if(operand < -1 || operand > 1)
	{
		return -1;
	}
	float theta1_s2 = -asinf(operand);


	*theta1 = atan2f(vx, vz);
	*theta2 = theta1_s2;
	return 0;
}


void get_ik_angles_double(double vx, double vy, double vz, double* theta1, double* theta2)
{
	double vx_pow2 = vx * vx;
	double vx_pow4 = vx_pow2 * vx_pow2;
	double vy_pow2 = vy * vy;
	double vz_pow2 = vz * vz;
	double vz_pow4 = vz_pow2 * vz_pow2;

	double operand = vx_pow4 + vx_pow2 * vy_pow2 + 2 * vx_pow2 * vz_pow2 + vy_pow2 * vz_pow2 + vz_pow4;
	double O2Targy_0 = -BasePlaneDistance * vy * vz / sqrt(operand);
	double O2Targz_0 = BasePlaneDistance * vx * vy / sqrt(vx_pow4 + vx_pow2 * vy_pow2 + 2 * vx_pow2 * vz_pow2 + vy_pow2 * vz_pow2 + vz_pow4) + BasePlaneDistance;

	double theta2_s2 = -asin((BasePlaneDistance - O2Targz_0) / BasePlaneDistance);
	double theta1_s2 = -asin(O2Targy_0 / (BasePlaneDistance * cos(theta2_s2)));

	*theta1 = atan2(vx, vz);
	*theta2 = theta1_s2;
}


