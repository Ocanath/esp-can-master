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
#include "dartt_mctl_params.h"
#include "dartt_gun_params.h"
#include "dartt.h"

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
extern buffer_t can_tx;

extern can_payload_t can_rx_mem;
extern buffer_t can_rx;

int send_fdcan_frame(uint16_t id, buffer_t * buffer);
int create_fdcan_struct_write_frame(
		buffer_t * field,
		buffer_t * device_mem,
		buffer_t * output_frame);
int write_fdcan_motor_int32_field(unsigned char * pfield, dartt_mctl_params_t * motor);
int write_fdcan_gun_int32_field(unsigned char * pfield, dartt_gun_params_t * gun);
int read_motor_reply(dartt_mctl_params_t * motor, uint32_t timeout);
int read_reply_blocking_fdcan_read(misc_read_message_t * read_msg, buffer_t * config_ref, uint32_t timeout);
int read_fdcan_motor_field(unsigned char * pfield, uint16_t num_bytes, dartt_mctl_params_t * motor);
int read_fdcan_gun_field(unsigned char * pfield, uint16_t num_bytes, dartt_gun_params_t * gun);

void FDCAN_Config(void);


#endif /* INC_FDCAN_H_ */
