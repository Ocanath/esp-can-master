#include "serial-comms.h"
#include "checksum.h"

const comms_t default_comms_t =
{
		.foc = {},		//init to zero
		.fds = {		//definitely do not init to zero
	       .module_number = 0,

	       /*The all important align offset*/
	       .align_offset_fixed = 0,

	       /*These are not hand obtained ever, so no commands for them*/
	       .is_flipped = 1,
	       .misc2 = {{0}},
	       .misc3 = {{0}},
	       .misc4 = {{0}},
	       .misc5 = {{0}},

	       /*every single one of these values needs a command. i might run out of room with an 8bit header...*/
	       .elec_conv_ratio_fixed = 11,
	       .gl_prop_delayloop_interval = 100,
	       .gl_prop_delay_const_12b = 0x7FFFFFFF,
	       .iq_pi = {
	               .kp = {
	                               .i32 = 0,
	                               .radix = 0
	               },
	               .ki = {
	                               .i32 = 0,
	                               .radix = 0
	               },
	               .x_integral_div = 1,    //this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
	               .x_sat = 0,
	               .out_rshift = 31
	       },
	       .id_pi = {
	               .kp = {
	                               .i32 = 0,
	                               .radix = 31
	               },
	               .ki = {
	                               .i32 = 0,
	                               .radix = 31
	               },
	               .x_integral_div = 1,
	               .x_sat = 0,
	               .out_rshift = 31
	       }
	},
	/*Initialize params for position control using FOC as 0*/
	.mpctl_rotor_iq = {
			.kpki =
			{
					.kp = {.i32 = 0, .radix = 0},
					.ki = {.i32 = 0, .radix = 0},
					.x = 0,
					.x_sat = 0,
					.out_rshift = 0
			},
			.kd = {.i32 = 0, .radix = 0},
			.out_sat = 0,
	},
	/*Initialize params for position control using sinusoidal as 0*/
	.mpctl_rotor_vq = {
			.kpki = {
					.kp = {.i32 = 40, .radix = 8},
					.ki = {.i32 = 0, .radix = 10},
					.x_integral_div = 130,
					.x = 0,	//no reason to init this to another value (unless you want the motor to jump when you start pctl... like a psycho...)
					.x_sat = 1000,	//no integral efforts for now
					.out_rshift = 0
			},
			.kd = {	//filter this will help for pos control. Need to do fixed point IIR
					.i32 = 5,	//no deriv effort either, for now
					.radix = 5
			},
			.out_sat = 2047,
	}
};

comms_t gl_mem = {};


/*
helper function to initialize the comms struct to the default values
 */
void init_comms(comms_t * comms)
{
	unsigned char * p_comms = (unsigned char *)comms;
	unsigned char * p_default_comms = (unsigned char *)&default_comms_t;

	for(int i = 0; i < sizeof(comms_t); i++)
	{
		*p_comms++ = *p_default_comms++;
	}
}

/*
Helper function to create a motor command.
INPUTS: address of motor
Returns:
	number of bytes written to msg
*/
int create_motor_command(unsigned char motor_address, int32_t command_word, unsigned char * msg, int msg_len)
{
	if(msg_len < NUM_BYTES_ADDRESS+sizeof(int32_t)+NUM_BYTES_CHECKSUM)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	int bidx = 0;
	msg[bidx++] = motor_address;
	int32_t * pcmd = (int32_t *)(&msg[bidx]);
	*pcmd = command_word;
	bidx += sizeof(int32_t);
	uint16_t * pchecksum = (uint16_t *)(&msg[bidx]);
	*pchecksum = get_crc16(msg, bidx);
	bidx += sizeof(uint16_t);
	return bidx;
}


/*
 * Parse a reply from a motor command
 * Motor commands are special: they consist of only an address, a context-dependent command word, and a checksum
 * The replies are always addressed to the master, and consist an address, cherry-picked motor values (iq, dtheta, theta_rotor) and a checksum 
 * This function parses the reply into the Master comms struct.
 * 
 * The master should maintain an instance of a comms struct for each motor
 * 
 */
int parse_motor_message_reply(unsigned char * msg, int msg_len, comms_t * comms)
{
	//zero'th byte is address, so we can ignore it
	int bidx = 1;
	const int expected_len = NUM_BYTES_ADDRESS + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t) + NUM_BYTES_CHECKSUM;
	if(msg_len != expected_len)
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	int checksum_index = (NUM_BYTES_ADDRESS + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t));
	if(get_crc16(msg, checksum_index) != *(uint16_t *)(&msg[checksum_index]))
	{
		return ERROR_CHECKSUM_MISMATCH;
	}

	//parse the first value (two bytes of gl_iq)
	int16_t * pi16;
	int32_t * pi32;
	pi16 = (int16_t *)(&msg[bidx]);
	bidx += sizeof(int16_t);
	comms->foc.gl_iq = (int32_t)(*pi16);

	pi16 = (int16_t *)(&msg[bidx]);
	bidx += sizeof(int16_t);
	comms->foc.gl_dtheta_fixedpoint_rad_p_sec = (int32_t)(*pi16);

	pi32 = (int32_t *)(&msg[bidx]);
	bidx += sizeof(int32_t);
	comms->foc.gl_theta_rem_m = *pi32;
	
	return SERIAL_PROTOCOL_SUCCESS;
}

/*Helper function to get the misc address from the motor address*/
unsigned char get_misc_address(unsigned char motor_address)
{
	return 0xFF - motor_address;
}

/*
    Parse an unstuffed (raw) message, including checksum and address splitting.
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed

    This function does address filtering and checksum validation
 */
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len,  comms_t * comms)
{   
    if(len < MINIMUM_MESSAGE_LENGTH)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
	//POTENTIAL VULN: assumes len is correct, and it may not be.
    uint16_t * pchecksum = (uint16_t *)(&(msg[len - sizeof(uint16_t)]));
    uint16_t checksum = get_crc16(msg, len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(msg[0] == motor_address)
        {
			/*
			Byte 0: motor address
			Byte 1-4: command word
			Byte 5-8: checksum
			*/
        	if(len < NUM_BYTES_ADDRESS+sizeof(int32_t)+NUM_BYTES_CHECKSUM)	//memory check for oaddnig the command word
        	{
        		return ERROR_MALFORMED_MESSAGE;
        	}
        	int32_t * p_cmd = (int32_t*)(&msg[1]);
        	comms->command_word = *p_cmd;	//load command word

        	if(replybuf_size < NUM_BYTES_ADDRESS + sizeof(int16_t) +sizeof(int16_t) + sizeof(int32_t) + NUM_BYTES_CHECKSUM)	//check that our intended reply will fit in the buffer
        	{
        		return ERROR_MALFORMED_MESSAGE;
        	}

        	int bidx = 0;   //index for the reply buffer

			//Consider: instead of always sending to master, we could send to the address of the motor that sent the message. 
			//We would want to make sure that the serial handler on slaves masks reads during self transmission, because
			//RS485 will put them on the bus. But this information could be more useful than addressing the master, as it 
			//helps us confirm that the message was sent by the intended motor.
        	p_replybuf[bidx++] = MASTER_ADDRESS;	//always send to master.

        	//load the first value (two bytes of gl_iq)
        	int16_t val_compressed_i16 = (int16_t)(comms->foc.gl_iq);
        	unsigned char * p_val = (unsigned char *)(&val_compressed_i16);
        	for(int i = 0; i < sizeof(val_compressed_i16); i++)
        	{
        		p_replybuf[bidx++] = p_val[i];
        	}

        	//load the second value (two bytes of velocity)
        	val_compressed_i16 = (int16_t)(comms->foc.gl_dtheta_fixedpoint_rad_p_sec);
        	p_val = (unsigned char *)(&val_compressed_i16);
        	for(int i = 0; i < sizeof(val_compressed_i16); i++)
        	{
        		p_replybuf[bidx++] = p_val[i];
        	}

        	//load the third value (4 bytes of gl_theta_rem_m)
        	p_val = (unsigned char *)(&comms->foc.gl_theta_rem_m);
        	for(int i = 0; i < sizeof(int32_t); i++)
        	{
        		p_replybuf[bidx++] = p_val[i];
        	}
			
			uint16_t checksum = get_crc16(p_replybuf, bidx);
			p_val = (unsigned char *)(&checksum);
			for(int i = 0; i < sizeof(uint16_t); i++)
			{
				p_replybuf[bidx++] = p_val[i];
			}
			*reply_len = bidx;

        	return SERIAL_PROTOCOL_SUCCESS;

        }
        else if(msg[0] == misc_address)
        {
            //remove address and checksum from the message and then parse
            msg = &(msg[1]);
            return parse_misc_command(msg, (len-(NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS)), p_replybuf, replybuf_size, reply_len, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}
