/*
 * m_uart.h
 *
 *  Created on: May 17, 2021
 *      Author: Ocanath
 */

#ifndef M_UART_H_
#define M_UART_H_
#include "main.h"
#include "stm32g4xx_it.h"
#include "cobs.h"
#include "dartt.h"

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
	DMA_Channel_TypeDef * rxdma;
	DMA_Channel_TypeDef * txdma;

	cobs_buf_t rx_mem;	//raw data buffer
	cobs_buf_t rx_decoded;	//cobs unstuffed

	buffer_t rx_decode_alias;
	payload_layer_msg_t rx_pld_msg;


	cobs_buf_t tx_mem;	//raw data buffer
	buffer_t tx_buf_alias;
}uart_it_t;



extern uart_it_t m_huart2;

//void m_uart_it_handler(uart_it_t * h);
void m_uart_it_handler(uart_it_t * h);
void m_uart_tx_start(uart_it_t * h, uint8_t * buf, int size);
void m_uart2_rx_cplt_callback(uart_it_t * h);
void m_uart_start_interrupts(uart_it_t * h);
void m_uart_rxdma_handler(DMA_HandleTypeDef *hdma);
void m_uart_txdma_handler(DMA_HandleTypeDef *hdma);
void m_uart_enable_rx_interrupt(uart_it_t * h);
void m_uart_disable_rx_interrupt(uart_it_t * h);
void m_uart_dma_transmit(buffer_t * tx_buf);

#endif /* M_UART_H_ */
