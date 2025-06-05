#include "checksum.h"
#include "serial-comms.h"
#include "serial-motor-comms.h"
#include "unity.h"

void print_comms_t(const comms_t* comms);

void test_write_struct_mem(void)
{
	comms_t master_comms = {};
	comms_t slave_comms = {};
	init_comms(&slave_comms);
	master_comms.fds.module_number = 3;
	slave_comms.fds.module_number = 3;
	master_comms.mpctl_rotor_vq.kpki.kp.i32 = 1000;
	master_comms.mpctl_rotor_vq.kpki.kp.radix = 4;
	master_comms.mpctl_rotor_vq.kpki.ki.i32 = 44;
	master_comms.mpctl_rotor_vq.kpki.ki.radix = 9;
	master_comms.mpctl_rotor_vq.kpki.x_integral_div = 100;
	master_comms.mpctl_rotor_vq.kpki.x = 1;
	master_comms.mpctl_rotor_vq.kpki.x_sat = 1111;
	master_comms.mpctl_rotor_vq.kpki.out_rshift = 2;

	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.i32, slave_comms.mpctl_rotor_vq.kpki.kp.i32);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.radix, slave_comms.mpctl_rotor_vq.kpki.kp.radix);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.i32, slave_comms.mpctl_rotor_vq.kpki.ki.i32);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.radix, slave_comms.mpctl_rotor_vq.kpki.ki.radix);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_integral_div, slave_comms.mpctl_rotor_vq.kpki.x_integral_div);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x, slave_comms.mpctl_rotor_vq.kpki.x);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_sat, slave_comms.mpctl_rotor_vq.kpki.x_sat);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.out_rshift, slave_comms.mpctl_rotor_vq.kpki.out_rshift);

	TEST_ASSERT_EQUAL(master_comms.fds.module_number, slave_comms.fds.module_number);//make sure the addys match

	unsigned char msg[64] = {};
	unsigned char reply[64] = {};
	int reply_len = 0;
	int len = create_write_struct_mem_message(&master_comms.mpctl_rotor_vq.kpki.kp.i32, sizeof(int32_t), &master_comms, msg, sizeof(msg));
	TEST_ASSERT_GREATER_THAN(0, len);
	int rc = parse_motor_message(slave_comms.fds.module_number, get_misc_address(slave_comms.fds.module_number), msg, len, reply, sizeof(reply), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.i32, slave_comms.mpctl_rotor_vq.kpki.kp.i32);
	slave_comms.mpctl_rotor_vq.kpki.kp.i32 = 0;
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.i32, slave_comms.mpctl_rotor_vq.kpki.kp.i32);

	len = create_write_struct_mem_message(&master_comms.mpctl_rotor_vq.kpki.kp.i32, sizeof(pctl_params_t), &master_comms, msg, sizeof(msg));
	TEST_ASSERT_GREATER_THAN(0, len);
	rc = parse_motor_message(slave_comms.fds.module_number, get_misc_address(slave_comms.fds.module_number), msg, len, reply, sizeof(reply), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.i32, slave_comms.mpctl_rotor_vq.kpki.kp.i32);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.radix, slave_comms.mpctl_rotor_vq.kpki.kp.radix);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.i32, slave_comms.mpctl_rotor_vq.kpki.ki.i32);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.radix, slave_comms.mpctl_rotor_vq.kpki.ki.radix);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_integral_div, slave_comms.mpctl_rotor_vq.kpki.x_integral_div);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x, slave_comms.mpctl_rotor_vq.kpki.x);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_sat, slave_comms.mpctl_rotor_vq.kpki.x_sat);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.out_rshift, slave_comms.mpctl_rotor_vq.kpki.out_rshift);

}

void test_read_struct_mem(void)
{
	comms_t master_comms = {};
	comms_t slave_comms = {};
	init_comms(&slave_comms);
	master_comms.fds.module_number = 3;
	slave_comms.fds.module_number = 3;
	/*init the slave comms structure with all non-zeros this time. we'll be updating master with the read response*/
	slave_comms.mpctl_rotor_vq.kpki.kp.i32 = 1000;
	slave_comms.mpctl_rotor_vq.kpki.kp.radix = 4;
	slave_comms.mpctl_rotor_vq.kpki.ki.i32 = 44;
	slave_comms.mpctl_rotor_vq.kpki.ki.radix = 9;
	slave_comms.mpctl_rotor_vq.kpki.x_integral_div = 100;
	slave_comms.mpctl_rotor_vq.kpki.x = 1;
	slave_comms.mpctl_rotor_vq.kpki.x_sat = 1111;
	slave_comms.mpctl_rotor_vq.kpki.out_rshift = 2;
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.i32, slave_comms.mpctl_rotor_vq.kpki.kp.i32);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.radix, slave_comms.mpctl_rotor_vq.kpki.kp.radix);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.i32, slave_comms.mpctl_rotor_vq.kpki.ki.i32);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.radix, slave_comms.mpctl_rotor_vq.kpki.ki.radix);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_integral_div, slave_comms.mpctl_rotor_vq.kpki.x_integral_div);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x, slave_comms.mpctl_rotor_vq.kpki.x);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_sat, slave_comms.mpctl_rotor_vq.kpki.x_sat);
	TEST_ASSERT_NOT_EQUAL(master_comms.mpctl_rotor_vq.kpki.out_rshift, slave_comms.mpctl_rotor_vq.kpki.out_rshift);
	TEST_ASSERT_EQUAL(master_comms.fds.module_number, slave_comms.fds.module_number);//make sure the addys match
	unsigned char msg[64] = {};
	unsigned char reply[64] = {};
	int reply_len = 0;
	int len = create_read_struct_mem_message(&master_comms.mpctl_rotor_vq, sizeof(pctl_params_t), &master_comms, msg, sizeof(msg));
	TEST_ASSERT_GREATER_THAN(0,len);
	int rc = parse_motor_message(slave_comms.fds.module_number, get_misc_address(slave_comms.fds.module_number), msg, len, reply, sizeof(reply), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	rc = update_comms_with_read_reply(&master_comms.mpctl_rotor_vq, &master_comms, reply, &reply_len);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.i32, slave_comms.mpctl_rotor_vq.kpki.kp.i32);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.kp.radix, slave_comms.mpctl_rotor_vq.kpki.kp.radix);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.i32, slave_comms.mpctl_rotor_vq.kpki.ki.i32);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.ki.radix, slave_comms.mpctl_rotor_vq.kpki.ki.radix);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_integral_div, slave_comms.mpctl_rotor_vq.kpki.x_integral_div);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x, slave_comms.mpctl_rotor_vq.kpki.x);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.x_sat, slave_comms.mpctl_rotor_vq.kpki.x_sat);
	TEST_ASSERT_EQUAL(master_comms.mpctl_rotor_vq.kpki.out_rshift, slave_comms.mpctl_rotor_vq.kpki.out_rshift);
}


//TODO: make test_motor_comms_misc_read_command which is the dual of the test_motor_comms_misc_write_command
/*
Test: create a misc write message, send it to the slave, verify the slave received the message.
First test just checks by writing align offset fixed.
 */
void test_motor_comms_misc_write_read_command(void)
{
	unsigned char slave_address = 0x07;
	//create a valid motor message
	unsigned char master_tx_msg[10] = {};
	unsigned char misc_address = get_misc_address(slave_address);
	comms_t master_comms = {};
	int index = index_of_field(&master_comms.fds.align_offset_fixed, &master_comms);
	master_comms.fds.align_offset_fixed = 51351;
	int master_tx_msg_len = create_misc_write_message(misc_address, index, (unsigned char *)(&master_comms.fds.align_offset_fixed), sizeof(master_comms.fds.align_offset_fixed), master_tx_msg, sizeof(master_tx_msg));
	TEST_ASSERT_EQUAL(NUM_BYTES_ADDRESS + sizeof(uint16_t) + sizeof(master_comms.fds.align_offset_fixed) + sizeof(uint16_t), master_tx_msg_len);
	uint16_t * pchecksum = (uint16_t *)(&master_tx_msg[master_tx_msg_len - sizeof(uint16_t)]);
	uint16_t checksum = get_crc16(master_tx_msg, master_tx_msg_len - sizeof(uint16_t));
	TEST_ASSERT_EQUAL(checksum, *pchecksum);

	uint8_t reply[32] = {};
	int reply_len = 0;

	comms_t slave_comms;
	init_comms(&slave_comms);
	slave_comms.fds.module_number = slave_address;
	slave_comms.fds.align_offset_fixed = 0;	//initialize to 0

	//this will be COBS or PPP stuffed in the link layer, then unstuffed before delivered to the parse function
	int rc = parse_motor_message(slave_comms.fds.module_number, get_misc_address(slave_comms.fds.module_number), master_tx_msg, master_tx_msg_len, reply, sizeof(reply), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(master_comms.fds.align_offset_fixed, slave_comms.fds.align_offset_fixed);


	master_tx_msg_len = create_misc_read_message(misc_address, index_of_field(&master_comms.fds.align_offset_fixed, &master_comms), 1, master_tx_msg, sizeof(master_tx_msg));
	parse_motor_message(slave_comms.fds.module_number,
							get_misc_address(slave_comms.fds.module_number), 
							master_tx_msg, 
							master_tx_msg_len, 
							reply, 
							sizeof(reply),  
							&reply_len, 
							&slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	int32_t * p_reply = (int32_t *)(&reply[NUM_BYTES_ADDRESS]);
	TEST_ASSERT_EQUAL(master_comms.fds.align_offset_fixed, *p_reply);
}


void test_single_struct_write_read(void)
{
	comms_t master_comms = {};
	unsigned char slave_address = 0x07;	//our target
	comms_t slave_comms = {};
	init_comms(&slave_comms);
	slave_comms.foc.gl_iq = 123344;
	slave_comms.foc.gl_id = -50518;
	slave_comms.foc.gl_dtheta_fixedpoint_rad_p_sec = 105815;
	slave_comms.foc.gl_theta_rem_m = -1515;


	slave_comms.fds.module_number = slave_address;
	unsigned char master_tx_buf[32] = {};
	unsigned char slave_tx_buf[32] = {};
	int reply_len = 0;
	TEST_ASSERT_NOT_EQUAL(master_comms.foc.gl_iq, slave_comms.foc.gl_iq);

	int txlen = create_misc_read_message(get_misc_address(slave_address),
										 index_of_field(&master_comms.foc, &master_comms), 
										 sizeof(master_comms.foc)/sizeof(int32_t), 
										 master_tx_buf, 
										 sizeof(master_tx_buf));

	unsigned char misc_address = get_misc_address(slave_comms.fds.module_number);
	int rc = parse_motor_message(slave_comms.fds.module_number, 
							misc_address, 
							master_tx_buf, 
							txlen, 
							slave_tx_buf, 
							sizeof(slave_tx_buf), 
							&reply_len, 
							&slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(sizeof(master_comms.foc)+NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM, reply_len);
	unsigned char * p_master_comms = (unsigned char *)(&master_comms.foc);
	for(int i = 0; i < reply_len; i++)
	{
		p_master_comms[i] = slave_tx_buf[i+NUM_BYTES_ADDRESS];
	}
	unsigned char * p_slave_comms = (unsigned char *)(&slave_comms.foc);
	for(int i = 0; i < sizeof(foc_params_t); i++)
	{
		TEST_ASSERT_EQUAL(p_master_comms[i], p_slave_comms[i]);
	}
	TEST_ASSERT_EQUAL(master_comms.foc.gl_iq, slave_comms.foc.gl_iq);
}

void test_full_struct_write_read(void)
{

	//Make a master comms struct with garbage values, bytewise non-zero
	comms_t master_comms = {};
	TEST_ASSERT_EQUAL(0, sizeof(master_comms) % sizeof(int32_t));	//verify the comms struct is 32-bit aligned
	unsigned char * p_master_comms = (unsigned char *)(&master_comms);
	for(int i = 0; i < sizeof(comms_t); i++)	//create a bullshit master comms struct with unique nonzero values in each field
	{
		p_master_comms[i] = i+1;
	}

	//Make a slave comms struct initialized to 0
	comms_t slave_comms = {};

	//confirm every byte is unequal
	unsigned char * pmaster = (unsigned char *)(&master_comms);
	unsigned char * pslave = (unsigned char *)(&slave_comms);
	for(int i = 0; i < sizeof(comms_t); i++)
	{
		TEST_ASSERT_NOT_EQUAL(pmaster[i], pslave[i]);
	}
	//TODO: write out the entire structure and confirm each byte is equal after parsing
	
	const unsigned char address = 45;//random address
	unsigned char master_uart_tx_buffer[sizeof(comms_t)+NUM_BYTES_NON_PAYLOAD] = {};
	int master_uart_tx_msg_len = create_misc_write_message(get_misc_address(address), 
															0, 
															(unsigned char *)(&master_comms), 
															sizeof(comms_t), 
															master_uart_tx_buffer, 
															sizeof(master_uart_tx_buffer)
														);
	//master writes out over tx
	//slave reads from rx
	int slave_rx_msg_len = master_uart_tx_msg_len; //remap the tx buffer to be the rx buffer
	unsigned char * slave_uart_rx_buffer = master_uart_tx_buffer; //pretend to be the rx buffer

	int reply_len = 0;
	unsigned char slave_uart_tx_buffer[sizeof(comms_t)+NUM_BYTES_NON_PAYLOAD] = {};
	int rc = parse_motor_message(address, get_misc_address(address), slave_uart_rx_buffer, slave_rx_msg_len, slave_uart_tx_buffer, sizeof(slave_uart_tx_buffer), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);

	//confirm every byte is equal
	for(int i = 0; i < sizeof(comms_t); i++)
	{
		TEST_ASSERT_EQUAL(pmaster[i], pslave[i]);
	} 

	//read the structure back now
	master_uart_tx_msg_len = create_misc_read_message(get_misc_address(address), 0, sizeof(comms_t)/sizeof(int32_t), master_uart_tx_buffer, sizeof(master_uart_tx_buffer));
	slave_rx_msg_len = master_uart_tx_msg_len;
	rc = parse_motor_message(address, get_misc_address(address), slave_uart_rx_buffer, slave_rx_msg_len, slave_uart_tx_buffer, sizeof(slave_uart_tx_buffer), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	for(int i = 0; i < sizeof(comms_t); i++)
	{
		TEST_ASSERT_EQUAL(pmaster[i], slave_uart_tx_buffer[i+NUM_BYTES_ADDRESS]);
		TEST_ASSERT_NOT_EQUAL(0, slave_uart_tx_buffer[i+NUM_BYTES_ADDRESS]);
	}
}





void test_motor_comms_motor_command(void)
{
	comms_t slave_comms;
	init_comms(&slave_comms);


	slave_comms.fds.module_number = 0x07;
	slave_comms.foc.gl_iq = -1535;
	slave_comms.foc.gl_dtheta_fixedpoint_rad_p_sec = 8918;
	slave_comms.foc.gl_theta_rem_m = 5678;

	//create a valid motor message
	unsigned char msg[10] = {};
	int rc = create_motor_command(slave_comms.fds.module_number, 1234, msg, sizeof(msg));
	TEST_ASSERT_EQUAL(7, rc);
	TEST_ASSERT_EQUAL(0x07, msg[0]);
	int32_t * pcmd = (int32_t *)(&msg[1]);
	TEST_ASSERT_EQUAL(1234, *pcmd);
	uint16_t * pchecksum = (uint16_t *)(&msg[5]);
	uint16_t checksum = get_crc16(msg, 5);
	TEST_ASSERT_EQUAL(checksum, *pchecksum);

	uint8_t reply[32] = {};
	int reply_len = 0;
	//this will be COBS or PPP stuffed in the link layer, then unstuffed before delivered to the parse function
	rc = parse_motor_message(slave_comms.fds.module_number, get_misc_address(slave_comms.fds.module_number), msg, rc, reply, sizeof(reply), &reply_len, &slave_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(1234, slave_comms.command_word);
	comms_t master_comms = {};
	rc = parse_motor_message_reply(reply, reply_len, &master_comms);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(slave_comms.foc.gl_iq, master_comms.foc.gl_iq);
	TEST_ASSERT_EQUAL(slave_comms.foc.gl_dtheta_fixedpoint_rad_p_sec, master_comms.foc.gl_dtheta_fixedpoint_rad_p_sec);
	TEST_ASSERT_EQUAL(slave_comms.foc.gl_theta_rem_m, master_comms.foc.gl_theta_rem_m);

	//TODO: check for returned values
	//TODO: check returned length
}


void test_motor_comms_invalid_message_length(void)
{
	comms_t comms;//	unsigned char address = msg[bidx++];
	init_comms(&comms);

	unsigned char msg[6] = {};
	int rc = create_motor_command(0x07, 1234, msg, sizeof(msg));
	TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	TEST_ASSERT_NOT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
}


