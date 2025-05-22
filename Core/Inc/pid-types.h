/*
 * pid-types.h
 *
 *  Created on: May 22, 2025
 *      Author: ocanath
 *
 *      This header contains definitions for key types related to PID control
 */

#ifndef INC_PID_TYPES_H_
#define INC_PID_TYPES_H_
#include "misc-types.h"

typedef struct fixed_PI_params_t
{
	i32_t kp;
	i32_t ki;
	int32_t x_integral_div;	//no x in this structure. replace X from previous implementation with xdiv, and use external x parameter
	int32_t x_sat;
	uint8_t out_rshift;
}fixed_PI_params_t;

typedef struct fixed_PI_2_params_t
{
	i32_t kp;
	i32_t ki;
	int32_t x_integral_div;
	int32_t x;
	int32_t x_sat;
	uint8_t out_rshift;
}fixed_PI_2_params_t;

typedef struct pctl_params_t
{
	fixed_PI_2_params_t kpki;
	i32_t kd;
	int32_t out_sat;
}pctl_params_t;


#endif /* INC_PID_TYPES_H_ */
