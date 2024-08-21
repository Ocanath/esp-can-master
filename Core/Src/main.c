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

static int16_t can_tx_id = 0;
static uint8_t can_tx_buf[8] = {0};
static uint8_t trigger_can_tx = 0;

void ppp_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size == 12)	//2 address, 8 payload, 2 checksum
	{
		const uint8_t * pbu8 = (uint8_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		const uint16_t * pbu16 = (uint16_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		int i16_size = h->ppp_unstuffed_size / sizeof(int16_t);
		uint16_t checksum = fletchers_checksum16((uint16_t*)pbu16, i16_size - 1);
		if(checksum == pbu16[i16_size-1])
		{
			can_tx_id = pbu16[0];
			for(int i = 0; i < sizeof(can_tx_buf); i++)
			{
				can_tx_buf[i] = pbu8[i+sizeof(can_tx_id)];
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
			int len = PPP_stuff(can_tx_buf, sizeof(can_tx_buf), gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
			trigger_can_tx = 0;

		}


		if(tick - led_ts > 500)
		{
			led_ts = tick;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}
}

