#ifndef SERIAL_COMMS_STRUCT_H
#define SERIAL_COMMS_STRUCT_H
#include <stdint.h>
//any additional includes here for adding structs
#include "foc-types.h"

typedef enum {FOC_MODE, SINUSOIDAL_MODE, PCTL_IQ, PCTL_VQ, OPEN_LOOP_MODE} control_mode_t;	//foc with velocity?

/*
 * TODO: remove all separate initializations of these and replace them with dereferences to the gl_comms
 * */
typedef struct comms_t
{
    foc_params_t foc;	//field oriented control parameters
    fds_motor_params_t fds;	//filesystem ram structure

    pctl_params_t mpctl_rotor_iq;	//position control settings for iq based pctl
    pctl_params_t mpctl_rotor_vq;	//position control settings for vq based pctl

    uint32_t motor_command_mode;	//command mode

    uint32_t write_filesystem_flag;	//flag to trigger a write to the filesystem, updating settings contained in fds

    int32_t command_word;	//context-dependent command word. I.e. if motor_command_mode is in FOC mode, it gets parsed as q-axis current, if in position, it's qd_rotor, etc.

} comms_t;



#endif
