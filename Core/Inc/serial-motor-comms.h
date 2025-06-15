#ifndef SERIAL_MOTOR_COMMS_H
#define SERIAL_MOTOR_COMMS_H

#include "serial-comms.h"

extern comms_t gl_mem;

#define ERROR_TIMEOUT -5

void init_comms(comms_t * comms);
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, buffer_t * msg, buffer_t * reply, comms_t * comms);
int create_motor_command(unsigned char motor_address, int32_t command_word, buffer_t * msg);
unsigned char get_misc_address(unsigned char motor_address);
int parse_motor_message_reply(buffer_t * msg, comms_t * comms);
int create_write_struct_mem_message(void * pstart, size_t num_bytes, comms_t * pcomm, buffer_t * msg);
int create_read_struct_mem_message(void * pstart, size_t num_bytes, comms_t * pcomm, buffer_t * msg);
int update_comms_with_read_reply(void * pword, comms_t * pcomms, buffer_t * msg);

#endif





