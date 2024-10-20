#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"
#include "IIRsos.h"
#include "m_mcpy.h"

#define NUM_MOTORS 3

typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;

enum {POSITION = 0xFA, TURBO = 0xFB, STEALTH = 0xFC};

void m_uart2_rx_cplt_callback(uart_it_t * h)
{

}


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
			for(int i = 0; i < NUM_MOTORS; i++)
			{
				gl_crq.commands[i] = pbi32[i];
			}
			uart_buf_received = 1;
		}
	}
}

uint8_t firststuff[14*2 + 2];


void send_misc_u8(uint16_t id, uint8_t header, uint8_t val)
{
	for(int i = 0; i < 8; i++)
		can_tx_data.d[i] = 0;
	can_tx_data.d[0] = header;
	can_tx_data.d[1] = val;
	can_tx_header.Identifier = (0x7FF - id);	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
}

void send_misc_i32(uint16_t id, uint8_t header, int32_t val)
{
	for(int i = 0; i < 8; i++)
		can_tx_data.d[i] = 0;
	can_tx_data.d[0] = header;
	u32_fmt_t fmt;
	fmt.i32 = val;
	for(int i = 0; i < sizeof(int32_t); i++)
	{
		can_tx_data.d[i+1] = fmt.u8[i];
	}
	can_tx_data.d[1] = val;
	can_tx_header.Identifier = (0x7FF - id);	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
	HAL_Delay(1);	//delay to let the message go out
}

void send_motor_i32(uint16_t id, int32_t val)
{

	can_tx_data.i32[0] = val;
	can_tx_header.Identifier = id;	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
	while((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U);
}

typedef struct m_motor_t
{
	uint16_t id;
	int32_t can_command;

	int32_t position;
	int16_t current;
	int16_t velocity;

	uint8_t control_mode;
	//position control iq vq settings
	uint8_t enabled_uart_encoder;
	uint8_t led_state;
}m_motor_t;

static m_motor_t motors[NUM_MOTORS] =
{
			{
					.id = 1,
					.control_mode = SET_SINUSOIDAL_MODE,
					.led_state = 1
			},
			{
					.id = 2,
					.control_mode = SET_PCTL_VQ_MODE,
					.led_state = 1
			},
			{
					.id = 3,
					.control_mode = SET_PCTL_VQ_MODE,
					.led_state = 1
			}
};


//setport 6701

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

	uint32_t led_ts = 0;
	uint32_t can_tx_ts = 0;

	HAL_Delay(1000);

	int tx_ididx = 0;
	send_misc_u8(3, SET_PCTL_VQ_MODE, 0);
	send_misc_u8(2, SET_PCTL_VQ_MODE, 0);
	send_misc_u8(1, SET_PCTL_VQ_MODE, 0);

	send_misc_i32(3, CHANGE_PCTL_VQ_KP_VALUE, 30);
	send_misc_i32(3, CHANGE_PCTL_VQ_KP_RADIX, 5);
	send_misc_i32(3, CHANGE_PCTL_VQ_KI_VALUE, 1);
	send_misc_i32(3, CHANGE_PCTL_VQ_KI_RADIX, 10);
	send_misc_i32(3, CHANGE_PCTL_VQ_OUTSAT, 3000);

	send_misc_i32(motors[0].id, CHANGE_PCTL_VQ_OUTSAT, 3546);
	send_misc_i32(2, CHANGE_PCTL_VQ_OUTSAT, 3546);

	uint8_t trigger_can_tx = 0;
	uint32_t filterts = 0;
	iirSOS upsampling_filter[NUM_MOTORS] = {0};
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		m_mcpy(&upsampling_filter[i], &gl_upsampling_filter, sizeof(iirSOS));
	}

	while (1)
	{
		uint32_t tick = HAL_GetTick();


		/*Upsample the input signal:*/
		if(gl_crq.mode == POSITION || gl_crq.mode == STEALTH)
		{
			if( (tick - filterts) > 0)
			{
				filterts = tick;
				for(int i = 0; i < NUM_MOTORS; i++)
				{
					float cmd_in = (float)gl_crq.commands[i];
					float cmd_out = sos_f(&upsampling_filter[i], cmd_in);
					motors[i].can_command = (int32_t)cmd_out;
				}
			}
		}


		/*Handle comms*/
		if(uart_buf_received != 0)
		{
			uart_buf_received = 0;

			//mode with 1 byte of padding, position, checksum
			/*Blast out the motor data back to the person who asked us to move! client doesn't really need to parse it*/
			uint8_t prestuff[1*sizeof(int16_t) + sizeof(int32_t)*3 + 1*sizeof(int16_t)] = {0};	//length is currently fixed, but in future, if we continue with FD can, we will need to extend this.
			/*
			 * Byte 0: mode
			 * Byte 1: pad (zeo)
			 * Byte 2,3,4,5: motor data 32
			 * Byte 6,7,8,9: motor data 32
			 * Byte 10,11,12,13: motor data 32
			 * Bytes 14,15: checksum16
			 * */
			prestuff[0] = gl_crq.mode;
			uint16_t* pbu16 = (uint16_t*)&prestuff[0];
			int32_t * pbi32 = (int32_t*)(&prestuff[2]);
			if(gl_crq.mode == POSITION || gl_crq.mode == STEALTH)
			{
				pbi32[0] = motors[0].position;
				pbi32[1] = motors[1].position;
				pbi32[2] = motors[2].position;
			}
			else if(gl_crq.mode == TURBO)
			{
				pbi32[0] = motors[0].velocity;
				pbi32[1] = motors[1].velocity;
				pbi32[2] = motors[2].velocity;
			}
			pbu16[7] = fletchers_checksum16(pbu16, 7);

			int len = PPP_stuff(prestuff, sizeof(prestuff), firststuff, sizeof(firststuff));
			len = PPP_stuff(firststuff, len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
		}






		/*in-loop receive and transmit*/
		if((tick - can_tx_ts) > 10 || trigger_can_tx != 0)	//TODO: increase bandwidth by adding trigger for TX when we get a can RX
		{
			trigger_can_tx = 0;

			can_tx_ts = tick;
			//testval = sin_12b(wrap_2pi_12b(tick*10 + (PI_12B*i/3)))*4;
			send_motor_i32(motors[tx_ididx].id, motors[tx_ididx].can_command);
			tx_ididx = (tx_ididx + 1) % NUM_MOTORS;
		}
		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data.d);
			{
				trigger_can_tx = 1;	//if we received a reply before our pending timeout, trigger another tx  before the 10ms timeout!
				uint16_t id = can_rx_header.Identifier;//note, test this, should retrieve correct ID
				motors[id-1].position = can_rx_data.i32[0];
				motors[id-1].current = can_rx_data.i16[2];
				motors[id-1].velocity = can_rx_data.i16[3];
			}
		}


		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}
}

