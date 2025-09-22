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


static uint8_t gl_rx_mem[UART_IT_BUF_SIZE] = {};
static uint8_t gl_rx_decoded[UART_IT_BUF_SIZE] =  {};
static uint8_t gl_tx_mem[UART_IT_BUF_SIZE] = {};
static uint8_t gl_tx_decoded[UART_IT_BUF_SIZE] =  {};
/*Initialize a baremetal uart handler structure for UART 1*/
uart_it_t m_huart2 =
{
		.Instance = USART2,
		.dma = DMA1_Channel1,
		.rx_mem =
		{
				.buf = gl_rx_mem,
				.size = sizeof(gl_rx_mem),
				.length = 0
		},
		.rx_decoded =
		{
				.buf = gl_rx_decoded,
				.size = sizeof(gl_rx_decoded),
				.length = 0
		},
		.tx_mem =
		{
				.buf = gl_tx_mem,
				.size = sizeof(gl_tx_mem),
				.length = 0
		},
		.tx_decoded =
		{
				.buf = gl_tx_decoded,
				.size = sizeof(gl_tx_decoded),
				.length = 0
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
//	h->Instance->CR1 |= (1 << 5) | (1 << 7) | (1 << 2) | (1 << 3);       //enable rxneie, txeie, RE and TE
//	h->Instance->CR1 &= ~(1 << 7);       //disable TX interrupt
//	h->Instance->CR1 |= (1 << 4);        //enable IDLE interrupt
	h->Instance->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
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

	uint32_t isrflags   = h->Instance->ISR;	//read interrupt status register

	uint16_t rdr = (uint16_t)h->Instance->RDR;	//read RDR, thus clearing the associated interrupt flag

	int rxne = (isrflags & RXNE_BIT) != 0;		//check if there's bytes in the queue

	if(rxne != 0 && rdr == 0)	//if there's stuff in the buffer and that the stuff in the buffer has value zero
	{
		//reset the dma pointer back to zero. we received a COBS frame, so everything preceeding is irrelevant.
		h->dma->CCR &= ~DMA_CCR_EN;
		h->dma->CNDTR = m_huart2.rx_mem.size;	//may need to frame disable/enable
		h->dma->CCR |= DMA_CCR_EN;
		cobs_decode_double_buffer(&h->rx_mem, &h->rx_decoded);
	}

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
