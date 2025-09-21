/*
 * dartt-controller-params.c
 *
 *  Created on: Sep 20, 2025
 *      Author: Ocanath Robotman
 */


#include "dartt_controller_params.h"
#include "fds.h"

dartt_controller_params_t dp_ctl =
{
		.fds = {},	//initialized by load, so leave empty
		.motors_ctl = {
				{
						.fds_mp =
						{
								.module_number = 1
						},
				},
				{
						.fds_mp =
						{
								.module_number = 2
						}
				}
		},
		.gun_ctl =
		{
				.fds_mp =
				{
						.module_number = 0
				}
		},
		.update_fs = 0,
		.load_flags = 0
};

buffer_t dp_ctl_alias =
{
		.buf = (unsigned char *)(&dp_ctl),
		.size = sizeof(dartt_controller_params_t),
		.len = sizeof(dartt_controller_params_t)
};

buffer_t motor_ctl_command_alias[NUM_MOTORS] =
{
		{
				.buf = (unsigned char *)(&dp_ctl.motors_ctl[0].command_word),
				.size = sizeof(int32_t),
				.len = sizeof(int32_t)
		},
		{
				.buf = (unsigned char *)(&dp_ctl.motors_ctl[1].command_word),
				.size = sizeof(int32_t),
				.len = sizeof(int32_t)
		}
};

dartt_mctl_params_t motors_periph[NUM_MOTORS] =
{
		{
				.fds_mp =
				{
						.module_number = 1	//hardcode the module numbers
				},
		},
		{
				.fds_mp =
				{
						.module_number = 2
				},
		}
};	//init with correct id, so read_fdcan_motor_field addresses the correct motor

buffer_t fs_alias = {
		.buf = (unsigned char *)(&dp_ctl.fds),
		.size = sizeof(fs_params_t),
		.len = sizeof(fs_params_t)
};

dartt_gun_params_t gun_periph;


