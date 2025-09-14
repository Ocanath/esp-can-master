/*
 * dartt_controller_params.h
 *
 *  Created on: Sep 14, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_DARTT_CONTROLLER_PARAMS_H_
#define INC_DARTT_CONTROLLER_PARAMS_H_

#define NUM_MOTORS 2

//subset of the controller dartt parameters that are controlled by the nonvolatile storage controller
typedef struct fs_params_t
{
	int32_t motor_offsets[NUM_MOTORS];
}fs_params_t;

typedef struct dartt_controller_params_t
{
	fs_params_t fds;
	uint32_t update_fs;
	uint32_t load_flags;
}dartt_controller_params_t;


#endif /* INC_DARTT_CONTROLLER_PARAMS_H_ */
