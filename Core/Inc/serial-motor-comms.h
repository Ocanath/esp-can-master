#ifndef SERIAL_MOTOR_COMMS_H
#define SERIAL_MOTOR_COMMS_H

#include "serial-comms.h"

extern comms_t gl_mem;

void init_comms(comms_t * comms);
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len,  comms_t * comms);
int create_motor_command(unsigned char motor_address, int32_t command_word, unsigned char * msg, int msg_len);
unsigned char get_misc_address(unsigned char motor_address);
int parse_motor_message_reply(unsigned char * msg, int msg_len, comms_t * comms);
int create_read_struct_word_message(void * pstart, comms_t * pcomm, unsigned char * tx_buf, size_t tx_size);

#endif





