# DMA UART COBS Handler Strategy

## Purpose

A highly CPU offloaded, peripheral hardware driven DMA handler for high throughput RS485/USART (half duplex). The ultimate goal is to implement a COBS-only handler that works with extremely minimal or zero interrupt usage.

## Notes

One potential issue - using the global USART interrupt for null character/termination character checking with COBS is potentially problematic due to the fact that it conflicts with the DMA request offloading RDR to the target memory block. I believe that this is fundamentally incompatible.

Major issue at the moment - termination character catching for USART DMA.

There may be a method to efficiently check new DMA data for a termination character in the superloop, similar to FDCAN mailbox checking?

A DMA interrupt may be the way to go. More reading required.

DMA RDR should be equal to 0x40004424.

Circ resets to zero - it's just i = (i + 1) % size.

- That means that SIZE - CNDTR = (read len) until full
- then after filling, len = 64
- After filling, (index + 1) % size is the index of the beginning of the array
- I think it is easier to reset CNDTR in the interrupt handler on reception char. Disable DMA, set CNDTR to size,
re-enable DMA. Only a few writes, and once per frame is pretty decent to eliminate circular buffer logic. In this mode you would keep it in 'Normal' mode, so CNDTR can be handled properly. - note: this works. you def want to disable the half transfer complete interrupt though. - LOL you're implementing circular in an interrupt handler, it's the same thing dumbass. just take len = SIZE - CNDTR - same thing. you delete old data but that's okay.

## Implementation Strategy

### High Level Overview
The DMA UART handler will work as follows:
1. Enable DMA for USART receptions
2. Disable global USART interrupts but enable the USART DMA interrupt
3. In the USART DMA handler, check for termination characters. Upon reception, stop DMA transfers, decode buffer into global decode copy, then re-enable DMA transfers.
4. I believe the tranfer complete interrupt is the appropriate one here.

### DMA Interrupt Handler

Always clear DMA_ISR at the end of the handler. 

Enable only the TCE (transfer complete) with DMA_IFCR for the appropriate channel

Values to set in the DMA_CCR below. Unless specified, keep at reset value.

- MSIZE - set to buffer size
- PSIZE - 00 (default)
- MINC - 1 (from my understanding, this means increment by 1 for each transfer in memory until MSIZE is reached)
- CIRC? unsure
- DIR - 0 (def)
- TCIE - 1
- EN - 1 when done

CNDTR, CPAR, CMAR - per need


