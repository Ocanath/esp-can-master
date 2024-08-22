import socket
import time
from checksum import *
from PPP_stuffing import *

"""
This payload structure:
[msg type][actual can tx length][CAN message ID, 2 bytes][can payload, even number of bytes][16 bit fletcher's checksum]
"""
def create_can_payload(msg_type, tx_len, id, can_payload):
	payload = struct.pack('<b', msg_type)
	payload = payload + struct.pack('<b', tx_len)
	payload = payload + struct.pack('<H', id)
	if(len(can_payload) % 2 != 0):
		can_payload = can_payload + bytes(1)	#zero pad the payload so that we can calculate a fletcher's checksum. Worst case time is added 1 byte, but mostly we're sending 4 bytes anyways so it's a fine choice
	payload = payload + can_payload
	chk = fletchers_checksum16(payload)
	chk_bytes = struct.pack('<H', chk)
	payload = payload + chk_bytes
	return payload




if __name__ == "__main__":

	target_addr = ('192.168.123.69', 5670)
	server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	server_socket.settimeout(0.0) #make non blocking
	udp_server_addr = ('0.0.0.0', 5670)

	try:
		print("binding: "+udp_server_addr[0]+", "+str(udp_server_addr[1]))
		server_socket.bind(udp_server_addr)
		print("Bind successful")
	except:
		print("something blocked us from binding to this ip")


	ts = time.time()
	
	kill = False
	iter = 0
	while(kill == False):

		try:
			cur_time = time.time()
			if(cur_time - ts > 0.001):
				ts = cur_time
				
				iter = iter + 1
				if(iter > 9999):
					iter = 0
				str_iter = str(iter)
				can_payload = bytearray(str_iter,  encoding='utf8')

				# can_payload = bytearray(5)
				# can_payload[0] = ord('h')
				# can_payload[1] = ord('e')
				# can_payload[2] = ord('l')
				# can_payload[3] = ord('l')
				# can_payload[4] = ord('o')

				payload = create_can_payload(0, len(can_payload), 1, can_payload)
				pld_stuffed = PPP_stuff(payload)
				# print(pld_stuffed)

				server_socket.sendto(pld_stuffed, target_addr)

			try:
				pkt,source_addr = server_socket.recvfrom(512)
				print("From: "+source_addr[0]+":"+str(source_addr[1])+": ["+str(pkt) + "],  " + str(len(pkt)) + " bytes" )
			except BlockingIOError:
				pass
		except KeyboardInterrupt:
			kill=True


