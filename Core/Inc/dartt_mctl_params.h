/*
 * dartt_params.h
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_DARTT_MCTL_PARAMS_H_
#define INC_DARTT_MCTL_PARAMS_H_
#include "profiles.h"
#include "pctl.h"

typedef enum {FOC_MODE, SINUSOIDAL_MODE, PCTL_IQ, PCTL_VQ, OPEN_LOOP_MODE} control_mode_t;	//foc with velocity?

typedef struct dartt_mctl_params_t
{

	int32_t command_word;

	pctl_params_t mctl_iq;
	pctl_params_t mctl_vq;


	int32_t open_loop_vq;
	int32_t open_loop_vd;

	uint8_t en_blink_led;
	uint8_t use_uart_encoder;
	uint8_t control_mode;
	uint8_t led_state;

	fds_motor_params_t fds_mp;
	int32_t autocalibration_voltage;
	uint32_t load_dartt_flags;	//when in main loop, set to trigger a scan of all dartt flags. Wrapper to save compute in commutation for excess flag handling
	uint32_t do_open_loop_test;	//commute motor
	uint32_t auto_calibrate_align_offset;
	uint32_t update_fs;	//use uint32_t because it'll get padded anyway
}dartt_mctl_params_t;

extern dartt_mctl_params_t gl_dp;

#endif /* INC_DARTT_MCTL_PARAMS_H_ */
