/*
 * m_uart.h
 *
 *  Created on: May 17, 2021
 *      Author: Ocanath
 */

#ifndef M_UART_H_
#define M_UART_H_
#include "init.h"
#include "stm32g4xx_it.h"


#define UART_IT_BUF_SIZE 64		//fw generically capable of handling 24 bytes incoming.

/*The following structure is used to implement
 * the baremetal interrupt handler. The actual
 * handler should be populated with the handler function and a unique instance
 * of the uart_it_t structure.
 *
 *
 * */
typedef struct uart_it_t
{

	USART_TypeDef * Instance;

	int bytes_received;	//Set by handler, cleared by main software. main software should compare with 0 and use as a 'new frame receieved' flag
	int rx_idx;		//helper variable, used to increment through the rx buffer
	uint8_t rx_buf[UART_IT_BUF_SIZE];	//the actual rx data buffer

	int ppp_bidx;
	uint8_t ppp_rx_buf[UART_IT_BUF_SIZE*2+2];	//separate input buffer. STUFFED
	uint8_t ppp_unstuff_buf[UART_IT_BUF_SIZE];	//for unstuffing result
	int ppp_unstuffed_size;

	int bytes_to_send;	//Set by main software, cleared by handler.
	int tx_idx;	//helper variable, used to increment through the tx buffer
	uint8_t * tx_buf;	//pointer to txbuffer.
}uart_it_t;

extern uint8_t gl_ppp_stuff_buf[128];

extern uart_it_t m_huart2;

//void m_uart_it_handler(uart_it_t * h);
void m_uart_it_handler(uart_it_t * h, void (*callback)(uart_it_t * h) );
void m_uart_tx_start(uart_it_t * h, uint8_t * buf, int size);
void m_uart2_rx_cplt_callback(uart_it_t * h);
void ppp_rx_cplt_callback(uart_it_t * h);

#endif /* M_UART_H_ */
