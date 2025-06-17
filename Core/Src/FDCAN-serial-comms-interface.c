/*
 * FDCAN-serial-comms-interface.c
 *
 *  Created on: Jun 17, 2025
 *      Author: ocanath
 */
#include "FDCAN.h"
#include "serial-comms-FDCAN.h"

/*Wrapper function for sending a can frame*/
int send_can_frame(can_frame_t * frame)
{
	can_tx_header.Identifier = frame->id;
	can_tx_header.DataLength = (frame->length & 0xFF) << 16;	//shift the length up for the interface
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, frame->data);
}


/*Wrapper function for loading can frames into a can frame structure.
 *
 * This may serve as a template function if performance-optimized code is desired in the main loop.
 * */
int get_can_frame(can_frame_t * frame)
{
	if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
	{
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, frame->data);
		{
			frame->id = can_rx_header.Identifier;
			frame->length = can_rx_header.DataLength;
			return 0;
		}
	}
	return 1;
}
