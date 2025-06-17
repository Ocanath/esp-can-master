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
#include "uart_struct_comms.h"


/*This is the general comms handler*/
void ppp_uart1_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size > NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM)
	{
		gl_rx_unstuffed.len = h->ppp_unstuffed_size;
		gl_reply_received = 1;
	}

}


void ppp_uart2_rx_cplt_callback(uart_it_t * h)
{
	gl_wifi_msg.len = h->ppp_unstuffed_size;
	//general structure:
	//header (32bit, just for us to filter messages):
	//payload	(packed motor command)
	//checksum
}


/*
 * Saturate output
 * */
int32_t saturate(int32_t v, int32_t sat)
{
	if(v > sat)
		return sat;
	if(v < -sat)
		return -sat;
	return v;
}

comms_t gl_motors[2] = {};
uint32_t prev_command_mode[sizeof(gl_motors)/sizeof(comms_t)];
uint8_t gl_do_pctl = 0;
int32_t pctl_targs[sizeof(gl_motors)/sizeof(comms_t)] = {0};


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

	gl_motors[0].fds.module_number = 0x03;
	gl_motors[1].fds.module_number = 0x04;


//	/*TODO: turn this into a generalized function that writes, modifies, reads, and confirms consistency*/


	while (1)
	{

	}
}

