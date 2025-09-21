/*
 * init_motion.c
 *
 *  Created on: Sep 20, 2025
 *      Author: Ocanath Robotman
 */
//clean headers
#include "init_motion.h"
#include "trig_fixed.h"
#include "sin-math.h"
#include "dartt_mctl_params.h"
#include "dartt_controller_params.h"
#include "dartt.h"
#include "Smoothing.h"
#include "PPP.h"
//dirty headers
#include "FDCAN.h"
#include "m_uart.h"

void read_motor_memory(void)
{
	//initialize both command and peripheral buffers
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		//read the entire motor into our peripheral/read copy
		buffer_t periph_alias =
		{
				.buf = (unsigned char *)(&motors_periph[i]),
				.size = sizeof(dartt_mctl_params_t),
				.len = 0
		};	//buffer alias to the peripheral (read) struct copy
		for(int field = 0; field < sizeof(dartt_mctl_params_t); field += sizeof(int32_t)*2)
		{
			read_fdcan_motor_field(&(periph_alias.buf[field]), sizeof(int32_t)*2, &motors_periph[i]);	//read the whole memory in 8 byte chunks
		}

		//copy what we read to the command copy
		buffer_t command_alias =
		{
				.buf = (unsigned char *)(&dp_ctl.motors_ctl[i]),
				.size = sizeof(dartt_mctl_params_t),
				.len = 0
		};
		for(int b = 0; b < command_alias.size && b < periph_alias.size; b++)
		{
			command_alias.buf[b] = periph_alias.buf[b];
		}
	}
}

/*Helper for modifying position control settings*/
void write_pctl_settings(void)
{
	//write out pctl modification settings
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		dp_ctl.motors_ctl[i].mctl_vq.kpki.kp.i32 = 75;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.kp.i32), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.i32 = 2;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.i32), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.radix = 12;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.radix), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.x_integral_div = 45;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.x_integral_div), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kd.i32 = 5;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kd.i32), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].control_mode = PCTL_VQ;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].en_blink_led), &dp_ctl.motors_ctl[i]);	//note - have to use the 4 byte aligned address.
	}

}

/*init helper for turning on the position controllers
 *
 * The subroutine breaks down as follows:
 * 1. obtain the current motor position (this is done using motor command frames, but should use dartt frames)
 * 2. Take the current position (wrapped) and set it as the pctl target. Send the target
 * 3. clear the integral effort with a dartt command (x=0)
 * 4. immediately activate the position controller by setting outsat to a high value (300)
 *
 * */
void activate_motion(void)
{
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		send_fdcan_frame(dp_ctl.motors_ctl[i].fds_mp.module_number, &motor_ctl_command_alias[i]);
		read_motor_reply(&motors_periph[i], 1000);

		dp_ctl.motors_ctl[i].command_word = wrap_2pi_14b(motors_periph[i].theta_rem_m);
		send_fdcan_frame(dp_ctl.motors_ctl[i].fds_mp.module_number, &motor_ctl_command_alias[i]);
		read_motor_reply(&motors_periph[i], 1000);

		dp_ctl.motors_ctl[i].mctl_vq.kpki.x = 0;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.x), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.out_sat = 300;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.out_sat), &dp_ctl.motors_ctl[i]);
	}
}


static float qd[NUM_MOTORS] = {};
static smooth_mem_t sm[NUM_MOTORS]  = {};


/*Smoothly start wherever you are and move to zero,zero
 * dump telemetry over uart*/
void smooth_startup(void)
{
	//immediately start smoothing behavior
	float period = 5.f;
	uint32_t start_ts = HAL_GetTick();
	uint32_t tick = HAL_GetTick();
	uint32_t uart_tx_ts = 0;
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		init_smoothing_mem(&sm[i]);
	}

	while( (tick - start_ts) < (int32_t)(period*1000.f))
	{
		tick = HAL_GetTick();


		for(int i = 0; i < NUM_MOTORS; i++)
		{
			float q = ((float)motors_periph[i].theta_rem_m)/((float)(1<<14));
			smooth_qd(0.f, period, q, &sm[i], &qd[i], tick);

			dp_ctl.motors_ctl[i].command_word = wrap_2pi_14b(qd[i]*((float)(1<<14)) - dp_ctl.fds.motor_offsets[i]);	//todo: verify sign is correct

			send_fdcan_frame(dp_ctl.motors_ctl[i].fds_mp.module_number, &motor_ctl_command_alias[i]);
			read_motor_reply(&motors_periph[i], 1000);
		}

		if(tick - uart_tx_ts > 5 && m_huart2.bytes_to_send == 0)	//todo upgrade to cobs
		{
			uart_tx_ts = tick;
			uint32_t prestuff[5] = {0};	//motor1 pos, motor2 pos, fletcher's
			int fidx = 0;
			prestuff[fidx++] = wrap_2pi_14b(motors_periph[0].theta_rem_m - dp_ctl.fds.motor_offsets[0]); //sizeof(int32_t)*index + sizeof(int32_t) - 1
			prestuff[fidx++] = wrap_2pi_14b(motors_periph[1].theta_rem_m - dp_ctl.fds.motor_offsets[1]);
			prestuff[fidx++] = dp_ctl.motors_ctl[0].command_word;
			prestuff[fidx++] = dp_ctl.motors_ctl[1].command_word;
			prestuff[fidx++] = tick;

			int len = PPP_stuff((uint8_t*)(&prestuff[0]), sizeof(prestuff), gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
		}
	}
}
