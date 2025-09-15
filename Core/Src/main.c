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
//statically allocate buffer aliases to our copies of the motor control structures
buffer_t dartt_mctl_aliases[NUM_MOTORS] = {
		{
				.buf = (unsigned char *)(&motors[0]),
				.size = sizeof(dartt_mctl_params_t),
				.len = 0
		},
		{
				.buf = (unsigned char *)(&motors[1]),
				.size = sizeof(dartt_mctl_params_t),
				.len = 0
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




//DELETE - keeping for quick reference when writing DARTT over FDCAN
//void send_misc_u8(uint16_t id, uint8_t header, uint8_t val)
//{
//	for(int i = 0; i < 8; i++)
//		can_tx_data.d[i] = 0;
//	can_tx_data.d[0] = header;
//	can_tx_data.d[1] = val;
//	can_tx_header.Identifier = (0x7FF - id);	//0x7FF for misc commands
//	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
//	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
//}
//
//void send_misc_i32(uint16_t id, uint8_t header, int32_t val)
//{
//	for(int i = 0; i < 8; i++)
//		can_tx_data.d[i] = 0;
//	can_tx_data.d[0] = header;
//	u32_fmt_t fmt;
//	fmt.i32 = val;
//	for(int i = 0; i < sizeof(int32_t); i++)
//	{
//		can_tx_data.d[i+1] = fmt.u8[i];
//	}
//	can_tx_data.d[1] = val;
//	can_tx_header.Identifier = (0x7FF - id);	//0x7FF for misc commands
//	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
//	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
//	HAL_Delay(1);	//delay to let the message go out
//}
//

//this wrapper should be effective for dartt commands
//todo: Test with logic analyzer!
void send_motor_i32(uint16_t id, int32_t val)
{
//	can_tx_data.i32[0] = val;
	unsigned char * cpy_buf = (unsigned char *)(&val);
	for(int i = 0; i < sizeof(int32_t); i++)
	{
		can_tx.buf[i] = cpy_buf[i];
	}
	can_tx_header.Identifier = id;	//0x7FF for misc commands
	can_tx_header.DataLength = FDCAN_DLC_BYTES_4;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx.buf);
	while((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U);
}

/*
 * Generic can buffer send function
 * */
int send_fdcan_frame(uint16_t id, buffer_t * buffer)
{
	can_tx_header.Identifier = id;
	if(buffer->len > 0 && buffer->len <= 8)
	{
		can_tx_header.DataLength = (buffer->len & 0xF) << 16;
	}
	else if (buffer->len > 8)	//could build a function that uses division and modulo arithmetic to accomplish this but i believe this is more performant for short messages cus you fall thru the if statements
	{
		if(buffer->len == 12)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
		}
		else if(buffer->len == 16)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_16;
		}
		else if(buffer->len == 20)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
		}
		else if(buffer->len == 24)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_24;
		}
		else if(buffer->len == 32)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_32;
		}
		else if(buffer->len == 48)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_48;
		}
		else if(buffer->len == 64)
		{
			can_tx_header.DataLength = FDCAN_DLC_BYTES_64;
		}
	}
	else
	{
		return ERROR_INVALID_ARGUMENT;
	}

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, buffer->buf);
	while((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U);

	return SERIAL_PROTOCOL_SUCCESS;
}

//typedef struct m_motor_t
//{
//	uint16_t id;
//	int32_t can_command;
//
//	int32_t position;
//	int16_t current;
//	int16_t velocity;
//
//	uint8_t control_mode;
//	//position control iq vq settings
//	uint8_t enabled_uart_encoder;
//	uint8_t led_state;
//}m_motor_t;
//
//static m_motor_t motors[NUM_MOTORS] =
//{
//			{
//					.id = 7,
//					.control_mode = SET_SINUSOIDAL_MODE,
//					.led_state = 1
//			},
//			{
//					.id = 8,
//					.control_mode = SET_SINUSOIDAL_MODE,
//					.led_state = 1
//			}
//};


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


	while(1)
	{
		int i = 0;
		motors[i].open_loop_vd++;


		buffer_t field =
		{
				.buf = (unsigned char *)(&motors[i].open_loop_vd),
				.size = sizeof(int32_t),
				.len = sizeof(int32_t)
		};
		if(create_fdcan_struct_write_frame(&field, &dartt_mctl_aliases[i], &can_tx) == SERIAL_PROTOCOL_SUCCESS)
		{
			send_fdcan_frame(dartt_get_complementary_address(motors[i].fds_mp.module_number), &can_tx);
		}
//		send_motor_i32(motors[i].fds_mp.module_number, val++);
		HAL_Delay(1000);
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

