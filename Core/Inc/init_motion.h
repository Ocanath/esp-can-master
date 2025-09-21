/*
 * init_motion.h
 *
 *  Created on: Sep 20, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_INIT_MOTION_H_
#define INC_INIT_MOTION_H_
#include <stdint.h>

void read_gun_memory(void);
void read_motor_memory(void);
void write_pctl_settings(void);
void activate_motion(void);
void smooth_startup(void);
void stream_plotter_data(uint32_t tick);


#endif /* INC_INIT_MOTION_H_ */
