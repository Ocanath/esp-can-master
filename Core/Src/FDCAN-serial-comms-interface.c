/*
 * FDCAN-serial-comms-interface.c
 *
 *  Created on: Jun 17, 2025
 *      Author: ocanath
 */
#include "FDCAN.h"
#include "serial-comms-FDCAN.h"

int send_can_frame(can_frame_t * frame)
{
	can_tx_header.Identifier = frame->id;
	can_tx_header.DataLength = (frame->length & 0xFF) << 16;	//shift the length up for the interface
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, frame->data);
}
