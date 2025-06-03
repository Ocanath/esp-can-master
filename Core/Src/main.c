#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"
#include "IIRsos.h"
#include "m_mcpy.h"
#include "sin-math.h"

int main(void)
{
 	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	m_uart_enable_interrupt_flags(&m_huart1);
	m_uart_enable_interrupt_flags(&m_huart2);
	MX_FDCAN1_Init();
	FDCAN_Config();

	uint32_t led_ts = 0;
	uint32_t uart_tx_ts = 0;
	while (1)
	{
		uint32_t tick = HAL_GetTick();

		/*Handle comms*/
		if(tick - uart_tx_ts > 5)
		{
			uart_tx_ts = tick;

			//mode with 1 byte of padding, position, checksum
			/*Blast out the motor data back to the person who asked us to move! client doesn't really need to parse it*/
			uint8_t prestuff[3*sizeof(int32_t)+1*sizeof(int16_t)] = {0};	//motor1 pos, motor2 pos, fletcher's
			/*
			* Bytes 0,1,2,3 - motor1 position
			* Bytes 4,5,6,7 - motor2 position
			* Bytes 8,9,10,11 - time ms
			 * Bytes 12,13: checksum16
			 * */
			int idx = 0;
			int32_t * pbi32 = (int32_t*)(&prestuff[0]);
			pbi32[idx++] = tick;

			int len = PPP_stuff(prestuff, idx*sizeof(int32_t), gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
		}

		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}
}

