/*
 * dartt_params.h
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_DARTT_GUN_PARAMS_H_
#define INC_DARTT_GUN_PARAMS_H_

#include "profiles.h"
#include "dartt.h"

typedef struct fds_gun_params_t
{
	/*CAN id. THIS. MUST. BE. FIRST.*/
	uint32_t module_number;	//foc module number
	uint32_t shot_time;	//time in ms for shot duration
	int32_t shot_duty;	//brushed duty for shot intensity
	uint32_t clip_size;
}fds_gun_params_t;


typedef struct dartt_gun_params_t
{
	int32_t shot_request; //on reception of this frame, add to shot queue
	int32_t shot_queue;	//(should) be read-only - state variable, decrements when a shot completes and increments by shot_request
	int32_t ia;	//phase current a
	int32_t ib;	//phase current b
	int32_t ic; //phase current c
	uint32_t led_state;

	int32_t vq_brushed;


	fds_gun_params_t fds_mp;
	uint32_t load_dartt_flags;	//when in main loop, set to trigger a scan of all dartt flags. Wrapper to save compute in commutation for excess flag handling
	uint32_t update_fs;	//use uint32_t because it'll get padded anyway
}dartt_gun_params_t;


#endif /* INC_DARTT_GUN_PARAMS_H_ */
