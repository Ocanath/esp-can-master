/*
 * foc-types.h
 *
 *  Created on: May 22, 2025
 *      Author: ocanath
 *
 *      This header contains typedef struct definitions for motor-controller specific structures
 */

#ifndef INC_FOC_TYPES_H_
#define INC_FOC_TYPES_H_
#include "pid-types.h"

/*Commutation-specific types, including iq current, rotor position, PI state information, rotor speed.*/
typedef struct foc_params_t
{
	int32_t gl_iq;
	int32_t gl_id;
	int32_t gl_theta_rem_m;
	int32_t gl_theta_rotations;
	int32_t gl_dtheta_fixedpoint_rad_p_sec;
	int32_t gl_x_iqpi;
	int32_t gl_x_idpi;
}foc_params_t;

/*Non-volatile settings, including:
 *
 * 		-Number of pole-pair
 * 		-calibration offset
 * 		-FOC PI settings
 * 		-module number (serial/CAN ID)
 * */
typedef struct fds_motor_params_t
{
	/*CAN id. THIS. MUST. BE. FIRST.*/
	uint32_t module_number;

	/*Properties of the motor which have to be measured and differ for each instance*/
	int32_t align_offset_fixed;
	int32_t is_flipped;	//misc 1-5 are encoder related, and obsolete. they do not apply when AMS encoder is used
	u32_fmt_t misc2;
	u32_fmt_t misc3;
	u32_fmt_t misc4;
	u32_fmt_t misc5;


	/*Properties of the motor which are either inherent or do not change between instances
	 * These must be written by the CAN master*/
	int32_t elec_conv_ratio_fixed;
	int32_t gl_prop_delayloop_interval;
	int32_t gl_prop_delay_const_12b;
	fixed_PI_params_t iq_pi;
	fixed_PI_params_t id_pi;

}fds_motor_params_t;


#endif /* INC_FOC_TYPES_H_ */
