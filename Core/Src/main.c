#include "init.h"
#include "m_uart.h"
#include "PPP.h"

void m_uart2_rx_cplt_callback(uart_it_t * h)
{

}

static uint8_t can_tx_buf[8] = {0};
static uint8_t trigger_can_tx = 0;

void ppp_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size != 0)
	{
		const uint8_t * pbu8 = (uint8_t*)(&m_huart2.ppp_unstuff_buf[0]);	//alias for this so it's easier to type
		int i = 0;
		for(i = 0; i < h->ppp_unstuffed_size && i < sizeof(can_tx_buf); i++)
		{
			can_tx_buf[i] = pbu8[i];
		}
		for(; i < sizeof(can_tx_buf); i++)
		{
			can_tx_buf[i] = 0;
		}
		trigger_can_tx = 1;
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

