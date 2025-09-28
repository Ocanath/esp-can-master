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
#include "dartt.h"


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


//todo - dartt reads
//todo - dartt sync fcn that takes
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
			float m0filt = sos_f(&upsampling_filter[0], (float)(-dp_ctl.m1_qd));
			float m1filt = sos_f(&upsampling_filter[1], (float)(-dp_ctl.m2_qd));

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
			//IMPORTANT NOTE: proper superloop handling means that after a decode call the DMA should be DISABLED, inside the interrupt handler,
			//then re-enabled here, in the superloop.
			//This is critically important to make sure the superloop doesnt check for a flagged message that is out of date due to
			//being in the middle of a cobs decode call.
			//it also logically makes sense to drop frames if you haven't gotten the chance to parse the last one.

			//shit - actually maybe what makes more sense is to frame the decode in interrupt handler around dma disable, then re-enable when reading.
			//then disable interrupts when catching the cobs message in superloop, and re-enable when done parsing dartt

			//ok yes. disable rx interrupt upon decode, then re-enable it here.
			//that means the decode copy is untouchable while we're processing a new encoded copy
			//we do run the risk of dropping a frame if the delimiter arrives in the dma buffer before we re-enable interrupts, but that's ok because if we disabled dma
			//too, it would filter the frame for missing data. We have a better chance of catching it from the delimiter if that arrives in time


			//we could also do dartt in the handler. That would simplify this greatly, at the expense of compute in an interrupt handler
			if(m_huart2.rx_decoded.buf[0] == dp_ctl.fds.dartt_address)
			{
				dartt_frame_to_payload(&m_huart2.rx_decode_alias, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &m_huart2.rx_pld_msg);
				dartt_parse_general_message(&m_huart2.rx_pld_msg, TYPE_SERIAL_MESSAGE, &dp_ctl_alias, &m_huart2.tx_buf_alias);
				if(m_huart2.tx_buf_alias.len != 0)
				{
					cobs_encode_single_buffer(&m_huart2.tx_mem);
					m_uart_dma_transmit(&m_huart2);
				}
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

