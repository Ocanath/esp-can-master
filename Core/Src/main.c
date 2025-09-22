#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"
#include "IIRsos.h"
#include "m_mcpy.h"
#include "sin-math.h"
#include "dartt_mctl_params.h"
#include "dartt_controller_params.h"
#include "dartt.h"
#include "Smoothing.h"
#include "init_motion.h"


/**
 * TODO:
 * 1. implement master-copy motor-copy DARTT list where two separate copies are maintained - one which only gets updated by DARTT reads,
 * 		and one which only gets updated to whatever the master wants. Implement a subroutine that infrequently (once per second?) scans a
 * 		chunk of the master record and compares it to another junk of the slave record. If they mismatch, the master will write a chunk to the motor
 * 		from the master copy, then	read that chunk back from the motor to the motor copy. If they match, return success, and update to the next
 * 		chunk. Certain values must be ignored.
 *
 * 		This will allow us to make sure settings are updated simply by writing values to the master copy.
 *
 * 1.1 using this master-motor copy method, re-tune the position control gains. The current position controllers are unstable with the weapon
 * on.
 *
 * 2. Add motor control offsets to master fds/filesystem
 * 5. upgrade cobs handler to dma and make it the gold standard

 * OPTIONAL - more in scope for a new project, since this is not required for MVP functionality
 * 6. upgrade PC side comms to DARTT - can use ppp still
 * 7. upgrade all comms to COBS+DARTT
 *
 *
 *
 *
 */


//dartt_weapon_params_t weapon = {};	//todo: implement this. module number should be hardcoded to 3

typedef struct uart_can_request_t
{
	uint8_t mode;
	int32_t commands[NUM_MOTORS];
}uart_can_request_t;
uart_can_request_t gl_crq = {0};


unsigned char gl_test_copybuf[5] = {};

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_FDCAN1_Init();
	FDCAN_Config();
	load_flash_params(&fs_alias);

	read_motor_memory();
	read_gun_memory();
	write_pctl_settings();
	activate_motion();	//clean motion activation
	smooth_startup();	//startup subroutine - track to zero


	iirSOS upsampling_filter[NUM_MOTORS] = {0};
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		m_mcpy(&upsampling_filter[i], &gl_upsampling_filter, sizeof(iirSOS));
	}
	/*todo: initialize the filters by operating passively/no motion and passing motors_periph.theta_rem_m
	 * into the filter structure for a fixed amount of time (couple hundred ms?). Then, you set
	 * motors_ctl.command_word = motors_periph.theta_rem_m...? - problem occurs if you wrap theta_rem_m...
	 */
	uint32_t upsample_ts = 0;
	uint32_t led_ts = 0;
	while(1)
	{
		uint32_t tick = HAL_GetTick();

		if(tick - upsample_ts >= 1)
		{
			upsample_ts = tick;
			float m0filt = sos_f(&upsampling_filter[0], (float)(-gl_crq.commands[0]));
			float m1filt = sos_f(&upsampling_filter[1], (float)(-gl_crq.commands[1]));

			dp_ctl.motors_ctl[0].command_word = wrap_2pi_14b((int32_t)m0filt + dp_ctl.fds.motor_offsets[0]);	//todo: verify sign is correct
			dp_ctl.motors_ctl[1].command_word = wrap_2pi_14b((int32_t)m1filt + dp_ctl.fds.motor_offsets[1]);
		}

		//write and read, per motor, over fdcan!
		for(int i = 0; i < NUM_MOTORS; i++)
		{
			send_fdcan_frame(dp_ctl.motors_ctl[i].fds_mp.module_number, &motor_ctl_command_alias[i]);
			read_motor_reply(&motors_periph[i], 1000);
		}

		stream_plotter_data(tick);

		if(dp_ctl.load_flags != 0)
		{
			dp_ctl.load_flags = 0;
			if(dp_ctl.update_fs != 0)
			{
				dp_ctl.update_fs = 0;
				update_flash_params(&fs_alias);
			}
		}

		if(dp_ctl.gun_ctl.shot_request != 0)
		{
			write_fdcan_gun_int32_field((unsigned char *)(&dp_ctl.gun_ctl.shot_request), &(dp_ctl.gun_ctl));
			dp_ctl.gun_ctl.shot_request = 0;	//flag as handled for non-repeat shots
		}

		if(m_huart2.rx_decoded.length != 0)
		{
			//TODO: add dma disable and enable and re-test

			//dummy parse. proper method is pipe to dartt
			for(int i = 0; i < sizeof(gl_test_copybuf); i++)
			{
				gl_test_copybuf[i] = m_huart2.rx_decoded.buf[i];
			}
			m_huart2.rx_decoded.length = 0;
		}

		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}

}

