/*
 * sauron-eye-closedform-ik.c
 *
 *  Created on: Apr 11, 2025
 *      Author: ocanath
 */
#include "sin-math.h"

const float BasePlaneDistance = 35.f/2.f+19.84f;


//
float sqrtf(float x)
{
    /* Initial guess: you can choose x itself, or 1.0f, or something in between.
     * Starting at x is fine unless x is huge; convergence is quadratic anyway.
     */
    float y = x;

    /* Perform 3 Newton–Raphson iterations:
     *   y_{n+1} = 0.5 * (y_n + x / y_n)
     * Each iteration roughly doubles the correct bits.
     * After 3 it’s accurate to within about 1 ULP for IEEE single.
     */
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);

    return y;
}

/*
 *
 */
float arcsinf(float x)
{
    /* atan2 handles the sign and quadrant correctly */
    return atan2_approx(x, sqrtf(1.0 - x*x));
}


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
	float O2Targy_0 = -BasePlaneDistance*vy*vz/sqrtf(operand);
	operand = vx_pow4 + vx_pow2*vy_pow2 + 2*vx_pow2*vz_pow2 + vy_pow2*vz_pow2 + vz_pow4;
	if(operand < 0)
	{
		return -1;
	}
	float O2Targz_0 = BasePlaneDistance*vx*vy/sqrtf(operand) + BasePlaneDistance;

	//todo: pre-compute arcsin operand and check for -1 to 1 bounds
	operand = (BasePlaneDistance - O2Targz_0)/BasePlaneDistance;
	if(operand < -1 || operand > 1)
	{
		return -1;
	}
	float theta2_s2 = -arcsinf(operand);

	operand = O2Targy_0/(BasePlaneDistance*cos_fast(theta2_s2));
	if(operand < -1 || operand > 1)
	{
		return -1;
	}
	float theta1_s2 = -arcsinf(operand);


	*theta1 = atan2_approx(vx, vz);
	*theta2 = theta1_s2;
	return 0;
}

