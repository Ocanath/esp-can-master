#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"

typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;

void m_uart2_rx_cplt_callback(uart_it_t * h)
{

}


typedef struct uart_can_request_t
{
	uint8_t type;
	uint8_t len;
	int16_t can_tx_id;
	uint8_t can_tx_buf[8];

}uart_can_request_t;
uart_can_request_t gl_crq = {0};

static uint8_t trigger_can_tx = 0;

void ppp_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size != 0 && (h->ppp_unstuffed_size % 2) == 0)	//nonzero and even is our basic callback entry filter
	{
		const uint8_t * pbu8 = (uint8_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		const uint16_t * pbu16 = (uint16_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		int i16_size = h->ppp_unstuffed_size / sizeof(int16_t);	//even, so this is fine
		uint16_t checksum = fletchers_checksum16((uint16_t*)pbu16, i16_size - 1);	//checksum is always the last two bytes
		if(checksum == pbu16[i16_size-1])		//compare calculated against received
		{
			gl_crq.type = pbu8[0];
			gl_crq.len = pbu8[1];
			gl_crq.can_tx_id = pbu16[1];
			for(int i = 0; (i < gl_crq.len) && (i < sizeof(gl_crq.can_tx_buf)); i++)
			{
				gl_crq.can_tx_buf[i] = pbu8[i+4];	//start at 4, go until you hit either the requested length or run out of buffer to write to
			}
			trigger_can_tx = 1;
		}
	}
}

static int32_t ourmotors_position = 0;
static int16_t ourmotors_current = 0;
static int16_t ourmotors_velocity = 0;
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
}

void send_motor_i32(uint16_t id, int32_t val)
{

	can_tx_data.i32[0] = val;
	can_tx_header.Identifier = id;	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
}

static int32_t testval = 0;


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


	uint16_t idlist[] = {3,2,1};
	const int num_joints = sizeof(idlist)/sizeof(uint16_t);
	uint8_t led_state[sizeof(idlist)/sizeof(uint16_t)] = {0};
	int led_blink_idx = 0;
	send_misc_u8(3, SET_SINUSOIDAL_MODE, 0);
	send_misc_u8(2, SET_SINUSOIDAL_MODE, 0);
	send_misc_u8(1, SET_SINUSOIDAL_MODE, 0);

	while (1)
	{
		uint32_t tick = HAL_GetTick();
		if(trigger_can_tx)
		{
//			can_payload_t * pb_cpld = (can_payload_t*)(&gl_crq.can_tx_buf[0]);
		}

		if((tick - can_tx_ts) > 10 || trigger_can_tx != 0)	//TODO: increase bandwidth by adding trigger for TX when we get a can RX
		{
			trigger_can_tx = 0;

			can_tx_ts = tick;
			for(int i = 0; i < num_joints; i++)
			{
				testval = (sin_12b(wrap_2pi_12b(tick*10 + (PI_12B*i/3)))*1000)/4096;
				send_motor_i32(idlist[i], testval);
//				while((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U);
			}
		}

		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data.d);
			{
//				uint16_t id = can_rx_header.Identifier;//note, test this, should retrieve correct ID

				ourmotors_position = can_rx_data.i32[0];
				ourmotors_current = can_rx_data.i16[2];
				ourmotors_velocity = can_rx_data.i16[3];
				can_payload_t udp_tx_pld = {0};
				udp_tx_pld.i32[0] = -ourmotors_position;
				udp_tx_pld.i16[2] = -ourmotors_current;
				udp_tx_pld.i16[3] = -ourmotors_velocity;

				/*
				 * byte 0: msg format
				 * byte 1: length
				 * bytes 2-3: can identifier
				 * bytes 4-11: can payload content
				 * 	bytes 4,5,6,7/i32[1]: position
				 *	bytes 8,9: current
				 *	bytes 10,11: velocity
				 * bytes 12-13: fletcher's checksum				 *
				 **/
				uint8_t prestuff[14] = {0};	//length is currently fixed, but in future, if we continue with FD can, we will need to extend this.
				prestuff[0] = 0;
				prestuff[1] = 8;
				uint16_t* pb = (uint16_t*)&prestuff[0];
				pb[1] = 2;	//id of remote can target
				for(int i = 0; i < sizeof(udp_tx_pld.d); i++)
					prestuff[i+4] = udp_tx_pld.d[i];
				pb[6] = fletchers_checksum16(pb, 6);



				int len = PPP_stuff(prestuff, sizeof(prestuff), firststuff, sizeof(firststuff));
				len = PPP_stuff(firststuff, len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
				m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
			}
		}



		if(tick - led_ts > 100)
		{
			led_state[led_blink_idx] = (~led_state[led_blink_idx]) & 1;
			led_blink_idx = (led_blink_idx + 1) % num_joints;
//			for(int ididx = 0; ididx < 3; ididx++)
//			{
//				if(led_state[ididx] == 0)
//					send_misc_u8(idlist[ididx], LED_OFF, 0);
//				else
//					send_misc_u8(idlist[ididx] , LED_ON, 0);
//			}

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}
}

