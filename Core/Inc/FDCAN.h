/*
 * CAN.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_
#include "init.h"
#include "fds.h"

#define PAYLOAD_SIZE_CAN 64

#define FDCAN_READ_TIMEOUT 	1		//error code


typedef union
{
	uint8_t u8[PAYLOAD_SIZE_CAN];
	int32_t i32[PAYLOAD_SIZE_CAN/sizeof(int32_t)];	//all types are even multiples of 8, and sizeof evals at compile time so this is safe
	uint32_t ui32[PAYLOAD_SIZE_CAN/sizeof(uint32_t)];
	int16_t i16[PAYLOAD_SIZE_CAN/sizeof(int16_t)];
	float f32[PAYLOAD_SIZE_CAN/sizeof(float)];
//	double f64[PAYLOAD_SIZE_CAN/sizeof(double)];	//can include if use. 1 element array thing kind of skeeves me out so im commenting it
}can_payload_t;

extern FDCAN_TxHeaderTypeDef   can_tx_header;
extern FDCAN_RxHeaderTypeDef   can_rx_header;
extern uint32_t			can_tx_mailbox;

extern can_payload_t can_tx_mem;
//extern buffer_t can_tx;

extern can_payload_t can_rx_mem;
//extern buffer_t can_rx;

//int send_fdcan_frame(uint16_t id, buffer_t * buffer);

void FDCAN_Config(void);


#endif /* INC_FDCAN_H_ */
