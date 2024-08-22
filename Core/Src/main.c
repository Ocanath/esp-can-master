#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"

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

	uint32_t led_ts = 0;
	while (1)
	{
		uint32_t tick = HAL_GetTick();

		if(trigger_can_tx)
		{
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

			int len = PPP_stuff(gl_crq.can_tx_buf, gl_crq.len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
			trigger_can_tx = 0;
		}


		if(tick - led_ts > 10)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		}
	}
}

