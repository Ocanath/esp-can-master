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

dartt_mctl_params_t motors[NUM_MOTORS] = {
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
};

buffer_t motor_command_alias[NUM_MOTORS] = {
		{
				.buf = (unsigned char *)(&motors[0].command_word),
				.size = sizeof(int32_t),
				.len = sizeof(int32_t)
		},
		{
				.buf = (unsigned char *)(&motors[1].command_word),
				.size = sizeof(int32_t),
				.len = sizeof(int32_t)
		}
};

//dartt_weapon_params_t weapon = {};	//todo: implement this. module number should be hardcoded to 3


/*
 * TODO:
 *
 * We need to gut this codebase.
 * 1. Replace all UART handling of the FDCAN controller with DMA based, or at least interrupt based COBS handler (start with interrupt COBS-only and then move to DMA).
 * 2. Migrate the ESP32 to COBS-only UART
 * 4. Switch to DARTT FDCAN
 * 5. Switch PC control software to COBS+dartt for passthrough controls and UDP controls
 *
 * THE ULTIMATE GOAL: A FULL EMBEDDED DARTT PHYSICAL LAYER TRANSLATOR:
 * PC controller/master: issues a DARTT frame over either UART or UDP. If over UDP, issue a full UART DARTT frame with COBS framing so it gets forwarded by the ESP32. If UART, just issuea normal COBS DARTT frame
 * ESP32 - either passthrough over UART (super basic) or forwarding over UDP
 *
 *
 */

int32_t m1_velocitypos = 0;
int32_t m2_velocitypos = 0;
int32_t m1_velocity = 0;
int32_t m2_velocity = 0;

int32_t m0_targ14 = 0;
int32_t m1_targ14 = 0;

//setport 6701


/*Helper Function to create a struct based dartt write frame*/
int create_fdcan_struct_write_frame(
		buffer_t * field,
		buffer_t * device_mem,
		buffer_t * output_frame)
{
	// Calculate the field index using index_of_field
	int field_index = index_of_field((void*)field->buf, (void*)device_mem->buf, device_mem->size);
	if(field_index < 0)
	{
		return field_index; // Return the error code
	}
	if(field->len > device_mem->size)
	{
		return ERROR_MEMORY_OVERRUN;
	}

	// Create the write message
	misc_write_message_t write_msg = {
			.address = 0,	//ignore address
			.index = (uint16_t)field_index,
			.payload = {
					.buf = field->buf,
					.size = device_mem->size - field->len,
					.len = field->len
			}
	};

	// Create the frame (TYPE_SERIAL_MESSAGE only)
	return dartt_create_write_frame(&write_msg, TYPE_ADDR_CRC_MESSAGE, output_frame);
}

int write_fdcan_motor_int32_field(unsigned char * pfield, dartt_mctl_params_t * motor)
{
	buffer_t field =
	{
			.buf = pfield,
			.size = sizeof(int32_t),
			.len = sizeof(int32_t)
	};
	buffer_t alias =
	{
			.buf = (unsigned char *)(motor),
			.size = sizeof(dartt_mctl_params_t),
			.len = 0
	};
	if(create_fdcan_struct_write_frame(&field, &alias, &can_tx) == SERIAL_PROTOCOL_SUCCESS)
	{
		return send_fdcan_frame(dartt_get_complementary_address(motor->fds_mp.module_number), &can_tx);
	}
	else
	{
		return ERROR_INVALID_ARGUMENT;	//bad
	}
}



//lookup
uint8_t fdcan_lookup[] =
{
		0,
		1,
		2,
		3,
		4,
		5,
		6,
		7,
		8,
		12,
		16,
		20,
		24,
		32,
		48,
		64
};

int read_motor_reply(dartt_mctl_params_t * motor, uint32_t timeout)
{
	uint32_t start = HAL_GetTick();
	while((HAL_GetTick() - start) < timeout)
	{
		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx.buf);
			can_rx.len = (can_rx_header.DataLength >> 16) & 0xF;
			if(can_rx.len > 8)
			{
				if(can_rx.len < sizeof(fdcan_lookup))
				{
					can_rx.len = fdcan_lookup[can_rx.len];
				}
			}
			if(can_rx_header.Identifier == MASTER_MOTOR_ADDRESS)	//motor command - dartt specifies custom implementation
			{
				//this shouldn't happen, because we should only call this on dartt misc reads
				motor->theta_rem_m = can_rx_mem.i32[0];
				motor->iq = can_rx_mem.i16[2];	//note - this is divided by a number to get it to fit in a 16bit word - still fuzzy on how this works honestly, i forgor
				motor->dtheta_fixedpoint_rad_p_sec = can_rx_mem.i16[3];
				return 0;	//return on successful matching reply
			}
		}
	}
	return FDCAN_READ_TIMEOUT;
}

int read_reply_blocking_fdcan_read(misc_read_message_t * read_msg, buffer_t * config_ref, uint32_t timeout)
{
	uint32_t start = HAL_GetTick();
	while((HAL_GetTick() - start) < timeout)
	{
		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx.buf);
			can_rx.len = (can_rx_header.DataLength >> 16) & 0xF;
			if(can_rx.len > 8)
			{
				if(can_rx.len < sizeof(fdcan_lookup))
				{
					can_rx.len = fdcan_lookup[can_rx.len];
				}
			}
			if(can_rx_header.Identifier == MASTER_MOTOR_ADDRESS)	//motor command - dartt specifies custom implementation
			{
				//this shouldn't happen, because we should only call this on dartt misc reads
			}
			else if (can_rx_header.Identifier == MASTER_MISC_ADDRESS)
			{
				payload_layer_msg_t pld_msg = {};
			    dartt_frame_to_payload(&can_rx, TYPE_ADDR_CRC_MESSAGE, PAYLOAD_ALIAS, &pld_msg);	//calling this function is unnecessary - can just do manually if desired. keeping dartt lib usage consistent tho
				return dartt_parse_read_reply(&pld_msg, read_msg, config_ref);
			}
		}
	}
	return FDCAN_READ_TIMEOUT;
}

int read_fdcan_motor_field(unsigned char * pfield, uint16_t num_bytes, dartt_mctl_params_t * motor)
{
	int field_index = index_of_field(pfield, (unsigned char *)(motor), sizeof(dartt_mctl_params_t));
	if(field_index < 0)
	{
		return field_index;
	}
	misc_read_message_t read_msg = {};
	//ignore address
	read_msg.index = field_index;
	read_msg.num_bytes = num_bytes;	//numbytes is HERE! you can deploy another helper that does the same thing
	dartt_create_read_frame(&read_msg, TYPE_ADDR_CRC_MESSAGE, &can_tx);
	send_fdcan_frame(dartt_get_complementary_address(motor->fds_mp.module_number), &can_tx);
	buffer_t motor_alias =
	{
			.buf = (unsigned char *)(motor),
			.size = sizeof(dartt_mctl_params_t),
			.len = 0
	};
	return read_reply_blocking_fdcan_read(&read_msg, &motor_alias, 50);
}

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

	//create a buffer_t for can transmissions
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		buffer_t alias =
		{
				.buf = (unsigned char *)(&motors[i]),
				.size = sizeof(dartt_mctl_params_t),
				.len = 0
		};
		for(int field = 0; field < sizeof(dartt_mctl_params_t); field += sizeof(int32_t)*2)
		{
			gl_rc = read_fdcan_motor_field(&(alias.buf[field]), sizeof(int32_t)*2, &motors[i]);
		}
	}


	for(int i = 0; i < NUM_MOTORS; i++)
	{
		motors[i].mctl_vq.out_sat = 300;
		write_fdcan_motor_int32_field((unsigned char *)(&motors[i].mctl_vq.out_sat), &motors[i]);
		motors[i].control_mode = PCTL_VQ;
		write_fdcan_motor_int32_field((unsigned char *)(&motors[i].en_blink_led), &motors[i]);	//note - have to use the 4 byte aligned address.

//		motors[i].mctl_iq.out_sat = 1000;
//		write_fdcan_motor_int32_field((unsigned char *)(&motors[i].mctl_iq.out_sat), &motors[i]);
//		motors[i].control_mode = PCTL_IQ;
//		write_fdcan_motor_int32_field((unsigned char *)(&motors[i].en_blink_led), &motors[i]);	//note - have to use the 4 byte aligned address.

	}

	while(1)
	{
		uint32_t tick = HAL_GetTick();
//		motors[0].command_word = 0;
//		motors[1].command_word = 0;
		motors[0].command_word = sin_14b(wrap_2pi_14b(tick*10))*PI_14B/(1<<14);
		motors[1].command_word = cos_14b(wrap_2pi_14b(tick*10))*PI_14B/(1<<14);

		for(int i = 0; i < NUM_MOTORS; i++)
		{
			send_fdcan_frame(motors[i].fds_mp.module_number, &motor_command_alias[i]);
			read_motor_reply(&motors[i], 1000);
		}
	}

	//	send_motor_i32(motors[0].id, m0_offset);
	//	HAL_Delay(1);
	//	send_motor_i32(motors[1].id, m1_offset);
	//	HAL_Delay(1);
	//
	//	for(int i = 0; i < NUM_MOTORS; i++)
	//	{
	//		send_misc_i32(motors[i].id, CHANGE_PCTL_VQ_OUTSAT, 3546);
	//		HAL_Delay(1);
	////		send_misc_u8(motors[i].id, SET_PCTL_VQ_MODE, 0);	//load offsetted target (0) , then enable pctl_vq. also potentially change pctl gains
	//	}
	//
	//	uint8_t trigger_can_tx = 0;
	//
	//	iirSOS upsampling_filter[NUM_MOTORS] = {0};
	//	for(int i = 0; i < NUM_MOTORS; i++)
	//	{
	//		m_mcpy(&upsampling_filter[i], &gl_upsampling_filter, sizeof(iirSOS));
	//	}
	//	while (1)
	//	{
	//		uint32_t tick = HAL_GetTick();
	//
	//		motors[0].can_command = wrap_2pi_14b(gl_crq.commands[0] + m0_offset);	//todo: verify sign is correct
	//		motors[1].can_command = wrap_2pi_14b(gl_crq.commands[1] + m1_offset);
	//
	//		/*Handle comms*/
	//		if(uart_buf_received != 0)
	//		{
	//			uart_buf_received = 0;
	//			last_ppp_message_recieved_ts = tick;
	//			//mode with 1 byte of padding, position, checksum
	//			/*Blast out the motor data back to the person who asked us to move! client doesn't really need to parse it*/
	//			uint8_t prestuff[3*sizeof(int32_t)+1*sizeof(int16_t)] = {0};	//motor1 pos, motor2 pos, fletcher's
	//			/*
	//			* Bytes 0,1,2,3 - motor1 position
	//			* Bytes 4,5,6,7 - motor2 position
	//			* Bytes 8,9,10,11 - time ms
	//			 * Bytes 12,13: checksum16
	//			 * */
	//			int32_t * pbi32 = (int32_t*)(&prestuff[0]);
	//			uint16_t * pbu16 = (uint16_t*)(&prestuff[0]);
	//			pbi32[0] = motors[0].position; //sizeof(int32_t)*index + sizeof(int32_t) - 1
	//			pbi32[1] = motors[1].position;
	//			pbi32[2] = tick;
	//			pbu16[6] = get_crc16(prestuff, 12);
	//
	//			int len = PPP_stuff(prestuff, sizeof(prestuff), firststuff, sizeof(firststuff));
	//			len = PPP_stuff(firststuff, len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
	//			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
	//		}
	//
	//
	//		/*in-loop receive and transmit*/
	//		if((tick - can_tx_ts) > 10 || trigger_can_tx != 0)	//TODO: increase bandwidth by adding trigger for TX when we get a can RX
	//		{
	//			trigger_can_tx = 0;
	//
	//			can_tx_ts = tick;
	//			//testval = sin_12b(wrap_2pi_12b(tick*10 + (PI_12B*i/3)))*4;
	//			send_motor_i32(motors[tx_ididx].id, motors[tx_ididx].can_command);
	//			tx_ididx = (tx_ididx + 1) % NUM_MOTORS;
	//		}
	//
	//		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
	//		{
	//			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data.d);
	//			{
	//				trigger_can_tx = 1;	//if we received a reply before our pending timeout, trigger another tx  before the 10ms timeout!
	//				uint16_t id = can_rx_header.Identifier;//note, test this, should retrieve correct ID
	//				motors[id-motors[0].id].position = can_rx_data.i32[0];	//
	//				motors[id-motors[0].id].current = can_rx_data.i16[2];
	//				motors[id-motors[0].id].velocity = can_rx_data.i16[3];
	//			}
	//		}
	//
	//
	//		/*LED blink*/
	//		if(tick - led_ts > 100)
	//		{
	//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
	//		}
	//	}
}

