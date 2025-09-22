/*
 * m_uart.c
 *
 *  Created on: May 17, 2021
 *      Author: Ocanath
 */
#include "m_uart.h"
#include "PPP.h"

/* Flag to clear ALL uart-associated interrupt requests, without clobbering reserved bits
 * (1 << 20) | (1 << 17) | (1 << 12) | (1 << 11) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0)
 */
#define ICR_CLEAR_ALL	0x00121BDF

/*ISR bits*/
#define RXNE_BIT 	(1 << 5)
#define TXE_BIT		(1 << 7)
#define IDLE_BIT	(1 << 4)

/*CR1 bits*/
#define TXEIE		(1 << 7)


static uint8_t rx_mem[UART_IT_BUF_SIZE] = {};
static uint8_t rx_decoded[UART_IT_BUF_SIZE] =  {};
static uint8_t tx_mem[UART_IT_BUF_SIZE] = {};
static uint8_t tx_decoded[UART_IT_BUF_SIZE] =  {};
/*Initialize a baremetal uart handler structure for UART 1*/
uart_it_t m_huart2 =
{
		.Instance = USART2,
		.rx_mem =
		{
				.buf = rx_mem,
				.size = sizeof(rx_mem),
				.len = 0
		},
		.rx_decoded =
		{
				.buf = rx_decoded,
				.size = sizeof(rx_decoded),
				.len = 0
		},
		.tx_mem =
		{
				.buf = tx_mem,
				.size = sizeof(tx_mem),
				.len = 0
		},
		.tx_decoded =
		{
				.buf = tx_decoded,
				.size = sizeof(tx_decoded),
				.len = 0
		}
};

/**
  */
__weak void m_uart2_rx_cplt_callback(uart_it_t * h)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(h);
}


void m_uart_start_interrupts(uart_it_t * h)
{
	h->Instance->CR1 |= (1 << 5) | (1 << 7) | (1 << 2) | (1 << 3);       //enable rxneie, txeie, RE and TE
	h->Instance->CR1 &= ~(1 << 7);       //disable TX interrupt
	h->Instance->CR1 |= (1 << 4);        //enable IDLE interrupt
}

/*
 * Baremetal uart handler.
 *
 * Note: may require timer to trigger based on rx activity, to reset if partial frame detected. Depends on the behavior of the IDLE interrupt in
 * edge cases.
 *
 * Idea: simultaneously do PPP unstuffing
 * */
void m_uart_it_handler(uart_it_t * h)
{

//	uint32_t isrflags   = h->Instance->ISR;	//read interrupt status register
//
//	uint16_t rdr = (uint16_t)h->Instance->RDR;	//read RDR, thus clearing the associated interrupt flag
//
//	int rxne = (isrflags & RXNE_BIT) != 0;		//check if there's bytes in the queue
//
//	if(rxne != 0)	//if there's stuff in the buffer
//	{
//		uint8_t nb = rdr & 0x00FF;
//	}

	h->Instance->ICR |=  ICR_CLEAR_ALL;	//clear all remaining interrupt flags to avoid a storm
}


/*m_uart dma handler*/
void m_uart_dma_handler(DMA_HandleTypeDef *hdma)
{
    hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU));	//global per-channel interrupt clear
}



void m_uart_tx_start(uart_it_t * h, uint8_t * buf, int size)
{
}
