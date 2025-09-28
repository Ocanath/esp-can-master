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

/*Initialize a baremetal uart handler structure for UART 1*/
uart_it_t m_huart2 =
{
		.Instance = USART2,
		.rxdma = DMA1_Channel1,
		.rx_mem =
		{
				.buf = gl_rx_mem,
				.size = sizeof(gl_rx_mem),
				.length = 0,
				.encoded_state = COBS_ENCODED
		},
		.rx_decoded =
		{
				.buf = gl_rx_decoded,
				.size = sizeof(gl_rx_decoded),
				.length = 0,
				.encoded_state = COBS_DECODED
		},
		.rx_decode_alias =
		{
				.buf = gl_rx_decoded,
				.size = sizeof(gl_rx_decoded),
				.len = 0
		},
		.tx_mem =
		{
				.buf = gl_tx_mem,
				.size = sizeof(gl_tx_mem),
				.length = 0
		},
		.tx_buf_alias =
		{
				.buf = gl_tx_mem,
				.size = sizeof(gl_tx_mem),
				.len = 0
		},
		.rx_pld_msg = {}
};

/**
  */
__weak void m_uart2_rx_cplt_callback(uart_it_t * h)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(h);
}

/**/
void m_uart_start_interrupts(uart_it_t * h)
{
//	h->Instance->CR1 |= (1 << 5) | (1 << 7) | (1 << 2) | (1 << 3);       //enable rxneie, txeie, RE and TE
//	h->Instance->CR1 &= ~(1 << 7);       //disable TX interrupt
//	h->Instance->CR1 |= (1 << 4);        //enable IDLE interrupt

	h->Instance->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
	h->Instance->CR3 |= USART_CR3_DMAR;
	h->rxdma->CCR &= ~DMA_CCR_EN;	//disable dma (will often already be disabled. Necessary for writing to CNTR, etc.
	h->rxdma->CCR |= DMA_CCR_CIRC;
	h->rxdma->CNDTR = h->rx_mem.size;
	h->rxdma->CPAR = (uint32_t)(&h->Instance->RDR);
	h->rxdma->CMAR = (uint32_t)(&h->rx_mem.buf[0]);
	h->rxdma->CCR |= DMA_CCR_TCIE;
	h->rxdma->CCR |= DMA_CCR_EN;

}


/**/
void m_uart_disable_rx_interrupt(uart_it_t * h)
{
	h->Instance->CR1 &= ~USART_CR1_RXNEIE;
}

/**/
void m_uart_enable_rx_interrupt(uart_it_t * h)
{
	h->Instance->CR1 |= USART_CR1_RXNEIE;
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
	uint16_t rdr = (uint16_t)h->Instance->RDR;	//read RDR, thus clearing the associated interrupt flag
	if(rdr == 0)	//rxne will always be zero, because the DMA clears the FIFO. That means we don't care about the state of that bit - we only need to check RDR, or alternatively the most recent value in DMA memory
	{
		h->rx_mem.length = (h->rx_mem.size - (size_t)h->rxdma->CNDTR);	//load length based on dma register status. It counts down so we just reverse it from the known transfer size
		//reset the dma pointer back to zero. we received a COBS frame, so everything preceeding is irrelevant.
		h->rxdma->CCR &= ~DMA_CCR_EN;
		h->rxdma->CNDTR = h->rx_mem.size;	//may need to frame disable/enable
		h->rxdma->CCR |= DMA_CCR_EN;
		cobs_decode_double_buffer(&h->rx_mem, &h->rx_decoded);
		h->rx_decode_alias.len = h->rx_decoded.length; //dumb, but we have to copy the length because we have a dartt buffer and cobs buffer. Should really do something to unify these..
	}

	h->Instance->ICR |=  ICR_CLEAR_ALL;	//clear all remaining interrupt flags to avoid a storm
}

void m_uart_dma_transmit(buffer_t * tx_buf)
{
	return;//todo implement this function
}

/*m_uart dma handler*/
void m_uart_dma_handler(DMA_HandleTypeDef *hdma)
{
    hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU));	//global per-channel interrupt clear
}



void m_uart_tx_start(uart_it_t * h, uint8_t * buf, int size)
{
}
