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

#define PAYLOAD_SIZE_CAN 8
//no matter what, the HAL backend loads 8 bytes of CAN data from registers. So you better be
//ready to receive them

extern int32_t gl_v_uS_conv;	//used to calculate expression of rotor velocity in (2^radix)*radians/sec
extern int32_t gl_iq_rshift;

typedef enum {FOC_MODE, SINUSOIDAL_MODE, PCTL_IQ, PCTL_VQ, OPEN_LOOP_MODE} control_mode_t;	//foc with velocity?

enum {

	CALC_ALIGN_OFFSET = 0x1,
	CALC_ENC_MIDPOINTS = 0x2,


	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,

	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35,

	SET_FOC_MODE = 0x36,	//configure to foc mode. need to do once and it takes
	SET_SINUSOIDAL_MODE = 0x37,	//configure to velocity mode. send once and it takes
	CHANGE_V_RADIX = 0x38,	//radix for velocity expression setting
	CHANGE_IQ_RSHIFT = 0x39,	//rightshift for iq expression setting

	SET_PCTL_IQ_MODE = 0x40,	//configure to position control using FOC mode. send once and it takes. Needs all position settings written to do anything (stable)
	CHANGE_PCTL_IQ_KP_VALUE = 0x41,
	CHANGE_PCTL_IQ_KP_RADIX = 0x42,
	CHANGE_PCTL_IQ_KI_VALUE = 0x43,
	CHANGE_PCTL_IQ_KI_RADIX = 0x44,
	CHANGE_PCTL_IQ_KD_VALUE = 0x45,
	CHANGE_PCTL_IQ_KD_RADIX = 0x46,
	CHANGE_PCTL_IQ_XSAT = 0x47,
	CHANGE_PCTL_IQ_OUTSAT = 0x48,
	CHANGE_PCTL_IQ_OUT_RSHIFT = 0x49,


	SET_PCTL_VQ_MODE = 0x4A,	//configure to position control using sinusoidal mode. send once and it takes. Needs all position settings written to do anything (stable)
	CHANGE_PCTL_VQ_KP_VALUE = 0x4B,
	CHANGE_PCTL_VQ_KP_RADIX = 0x4C,
	CHANGE_PCTL_VQ_KI_VALUE = 0x4D,
	CHANGE_PCTL_VQ_KI_RADIX = 0x4E,
	CHANGE_PCTL_VQ_KD_VALUE = 0x4F,
	CHANGE_PCTL_VQ_KD_RADIX = 0x50,
	CHANGE_PCTL_VQ_XSAT = 0x51,
	CHANGE_PCTL_VQ_OUTSAT = 0x52,
	CHANGE_PCTL_VQ_OUT_RSHIFT = 0x53
};

typedef union
{
	uint8_t d[PAYLOAD_SIZE_CAN];
	int32_t i32[PAYLOAD_SIZE_CAN/sizeof(int32_t)];	//all types are even multiples of 8, and sizeof evals at compile time so this is safe
	uint32_t ui32[PAYLOAD_SIZE_CAN/sizeof(uint32_t)];
	int16_t i16[PAYLOAD_SIZE_CAN/sizeof(int16_t)];
	float f32[PAYLOAD_SIZE_CAN/sizeof(float)];
//	double f64[PAYLOAD_SIZE_CAN/sizeof(double)];	//can include if use. 1 element array thing kind of skeeves me out so im commenting it
}can_payload_t;

typedef union
{
	int16_t v;
	uint8_t d[sizeof(int16_t)];
}int16_fmt_t;

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

typedef union
{
	int32_t v;
	uint8_t d[sizeof(int32_t)];
}int32_fmt_t;

typedef union floatsend_t
{
	float v;
	uint8_t d[sizeof(float)];
}floatsend_t;

typedef union floatsend_chk_t
{
	float v;
	uint8_t d[sizeof(float) + 1];
}floatsend_chk_t;


extern uint16_t gl_ext_cmd_id;

extern FDCAN_TxHeaderTypeDef   can_tx_header;
extern FDCAN_RxHeaderTypeDef   can_rx_header;
extern can_payload_t 			can_tx_data;
extern can_payload_t 			can_rx_data;
extern uint32_t			can_tx_mailbox;

void FDCAN_Config(void);


#endif /* INC_FDCAN_H_ */
