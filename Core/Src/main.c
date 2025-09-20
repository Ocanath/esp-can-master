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

//dartt_mctl_params_t motors[NUM_MOTORS] = {
//		{
//				.fds_mp =
//				{
//						.module_number = 1	//hardcode the module numbers
//				},
//		},
//		{
//				.fds_mp =
//				{
//						.module_number = 2
//				},
//		}
//};

//buffer_t motor_command_alias[NUM_MOTORS] = {
//		{
//				.buf = (unsigned char *)(&motors[0].command_word),
//				.size = sizeof(int32_t),
//				.len = sizeof(int32_t)
//		},
//		{
//				.buf = (unsigned char *)(&motors[1].command_word),
//				.size = sizeof(int32_t),
//				.len = sizeof(int32_t)
//		}
//};

//dartt_weapon_params_t weapon = {};	//todo: implement this. module number should be hardcoded to 3

typedef struct uart_can_request_t
{
	uint8_t mode;
	int32_t commands[NUM_MOTORS];
}uart_can_request_t;
uart_can_request_t gl_crq = {0};
static uint8_t uart_buf_received = 0;

void ppp_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size != 0 && (h->ppp_unstuffed_size % 2) == 0)	//nonzero and even is our basic callback entry filter
	{
		const uint8_t * pbu8 = (uint8_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		const uint16_t * pbu16 = (uint16_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		const int32_t * pbi32 = (int32_t * )(&m_huart2.ppp_unstuff_buf[2]);	//alias for section of payload corresponding to 32bit target values
		int i16_size = h->ppp_unstuffed_size / sizeof(int16_t);	//must always be even, so this is fine
		uint16_t checksum = fletchers_checksum16((uint16_t*)pbu16, i16_size - 1);	//checksum is always the last two bytes
		if(checksum == pbu16[i16_size-1])		//compare calculated against received
		{
			/*
			 * 0: mode
			 * 1: pad
			 * 2-3-4-5: w1
			 * 6-7-8-9: w2
			 * 10-11-12-13: w3
			 * 14-15: chk
			 */
			gl_crq.mode = pbu8[0];
			for(int i = 0; i < NUM_MOTORS && i*sizeof(int32_t) < sizeof(m_huart2.ppp_unstuff_buf); i++)
			{
				gl_crq.commands[i] = pbi32[i];
			}
			uart_buf_received = 1;
		}
	}
}

uint8_t firststuff[14*2 + 2];

int gl_rc = 0;
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_FDCAN1_Init();
	FDCAN_Config();


	HAL_Delay(1000);

	//	int32_t m0_offset = -990;
	//	int32_t m1_offset = 16921;

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
			gl_rc = read_fdcan_motor_field(&(periph_alias.buf[field]), sizeof(int32_t)*2, &motors_periph[i]);	//read the whole memory in 8 byte chunks
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


	for(int i = 0; i < NUM_MOTORS; i++)
	{
		dp_ctl.motors_ctl[i].mctl_vq.out_sat = 300;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.out_sat), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.kp.i32 = 150;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.kp.i32), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.i32 = 1;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.i32), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.radix = 12;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.ki.radix), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kpki.x_integral_div = 50;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kpki.x_integral_div), &dp_ctl.motors_ctl[i]);
		dp_ctl.motors_ctl[i].mctl_vq.kd.i32 = 15;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].mctl_vq.kd.i32), &dp_ctl.motors_ctl[i]);

		dp_ctl.motors_ctl[i].control_mode = PCTL_VQ;
		write_fdcan_motor_int32_field((unsigned char *)(&dp_ctl.motors_ctl[i].en_blink_led), &dp_ctl.motors_ctl[i]);	//note - have to use the 4 byte aligned address.

//		motors[i].mctl_iq.out_sat = 1000;
//		write_fdcan_motor_int32_field((unsigned char *)(&motors[i].mctl_iq.out_sat), &motors[i]);
//		motors[i].control_mode = PCTL_IQ;
//		write_fdcan_motor_int32_field((unsigned char *)(&motors[i].en_blink_led), &motors[i]);	//note - have to use the 4 byte aligned address.

	}

	int32_t m0_offset = 0;
	int32_t m1_offset = 0;	//TODO: obtain these experimentally, and also load them into the filesystem
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
//		motors[0].command_word = 0;
//		motors[1].command_word = 0;
//		motors[0].command_word = sin_14b(wrap_2pi_14b(tick*10))*PI_14B/(1<<14);
//		motors[1].command_word = cos_14b(wrap_2pi_14b(tick*10))*PI_14B/(1<<14);


		if(tick - upsample_ts >= 1)
		{
			upsample_ts = tick;
			float m0filt = sos_f(&upsampling_filter[0], (float)(-gl_crq.commands[0]));
			float m1filt = sos_f(&upsampling_filter[1], (float)(-gl_crq.commands[1]));

			dp_ctl.motors_ctl[0].command_word = wrap_2pi_14b((int32_t)m0filt + m0_offset);	//todo: verify sign is correct
			dp_ctl.motors_ctl[1].command_word = wrap_2pi_14b((int32_t)m1filt + m1_offset);
		}




		for(int i = 0; i < NUM_MOTORS; i++)
		{
			send_fdcan_frame(dp_ctl.motors_ctl[i].fds_mp.module_number, &motor_ctl_command_alias[i]);
			read_motor_reply(&motors_periph[i], 1000);
		}


		/*Handle comms*/
		if(uart_buf_received != 0)
		{
			uart_buf_received = 0;
//			last_ppp_message_recieved_ts = tick;
			//mode with 1 byte of padding, position, checksum
			/*Blast out the motor data back to the person who asked us to move! client doesn't really need to parse it*/
			uint8_t prestuff[3*sizeof(int32_t)+1*sizeof(int16_t)] = {0};	//motor1 pos, motor2 pos, fletcher's
			/*
			* Bytes 0,1,2,3 - motor1 position
			* Bytes 4,5,6,7 - motor2 position
			* Bytes 8,9,10,11 - time ms
			 * Bytes 12,13: checksum16
			 * */
			int32_t * pbi32 = (int32_t*)(&prestuff[0]);
			uint16_t * pbu16 = (uint16_t*)(&prestuff[0]);
			pbi32[0] = motors_periph[0].theta_rem_m; //sizeof(int32_t)*index + sizeof(int32_t) - 1
			pbi32[1] = motors_periph[1].theta_rem_m;
			pbi32[2] = tick;
			pbu16[6] = fletchers_checksum16(pbu16, 6);

			int len = PPP_stuff(prestuff, sizeof(prestuff), firststuff, sizeof(firststuff));
			len = PPP_stuff(firststuff, len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
		}

		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}

}

