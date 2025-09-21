/*
 * dartt_controller_params.h
 *
 *  Created on: Sep 14, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_DARTT_CONTROLLER_PARAMS_H_
#define INC_DARTT_CONTROLLER_PARAMS_H_
#include "dartt.h"
#include "dartt_mctl_params.h"

#define NUM_MOTORS 2

//subset of the controller dartt parameters that are controlled by the nonvolatile storage controller
typedef struct fs_params_t
{
	int32_t motor_offsets[NUM_MOTORS];			//load these as the value of thetat_rem_m at the desired zero position.
}fs_params_t;

typedef struct dartt_controller_params_t
{
	fs_params_t fds;
	dartt_mctl_params_t motors_ctl[NUM_MOTORS];
	uint32_t update_fs;
	uint32_t load_flags;
	//add controller copies of dartt_mctl_params_t, which are the internal record of our intended mctl structs
}dartt_controller_params_t;


//controller memory and aliases
extern dartt_controller_params_t dp_ctl;
extern buffer_t dp_ctl_alias;
extern buffer_t motor_ctl_command_alias[NUM_MOTORS];

//motor memory and aliases
extern dartt_mctl_params_t motors_periph[NUM_MOTORS];
extern buffer_t fs_alias;

#endif /* INC_DARTT_CONTROLLER_PARAMS_H_ */
