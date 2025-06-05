#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"
#include "IIRsos.h"
#include "m_mcpy.h"
#include "sin-math.h"
#include "serial-motor-comms.h"

uint8_t gl_msg_buf[62] = {};
uint8_t gl_reply_received = 0;

/*This is the general comms handler*/
void ppp_uart1_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size > NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM)
	{
		int checksum_idx = h->ppp_unstuffed_size - sizeof(uint16_t);
		uint16_t crc = get_crc16(h->ppp_unstuff_buf, checksum_idx);
		uint16_t * p_checksum = (uint16_t*)(&h->ppp_unstuff_buf[checksum_idx]);
		if(*p_checksum == crc)
		{
			gl_reply_received = 1;
		}
	}

}


void ppp_uart2_rx_cplt_callback(uart_it_t * h)
{

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
	MX_USART1_UART_Init();
	m_uart_enable_interrupt_flags(&m_huart1);
	m_uart_enable_interrupt_flags(&m_huart2);
	MX_FDCAN1_Init();
	FDCAN_Config();

	uint32_t led_ts = 0;
	uint32_t uart_tx_ts = 0;

	comms_t motor[1] = {};
	while (1)
	{
		uint32_t tick = HAL_GetTick();

		/*Handle comms*/
		if(tick - uart_tx_ts > 5)
		{
			uart_tx_ts = tick;

			int msg_len = create_motor_command(0x03, 1000, gl_msg_buf, sizeof(gl_msg_buf));
			int len = PPP_stuff(gl_msg_buf, msg_len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
		}
		if(gl_reply_received)
		{
			parse_motor_message_reply(m_huart1.ppp_unstuff_buf, m_huart1.ppp_unstuffed_size, &motor[0]);
		}

		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}
}

