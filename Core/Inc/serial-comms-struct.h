#ifndef SERIAL_COMMS_STRUCT_H
#define SERIAL_COMMS_STRUCT_H
#include <stdint.h>
//any additional includes here for adding structs
#include "foc-types.h"

/*
 * TODO: remove all separate initializations of these and replace them with dereferences to the gl_comms
 * */
typedef struct comms_t
{
    foc_params_t foc;	//field oriented control parameters
    fds_motor_params_t fds;	//filesystem ram structure

    pctl_params_t pctl_iq;	//position control settings for iq based pctl
    pctl_params_t pctl_vq;	//position control settings for vq based pctl

    uint32_t motor_command_mode;	//command mode

    uint32_t write_filesystem_flag;	//flag to trigger a write to the filesystem, updating settings contained in fds

} comms_t;

extern comms_t gl_mem;

#endif
