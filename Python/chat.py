import socket
import time
from checksum import *
from PPP_stuffing import *


def create_can_payload(id, can_payload):

	id_bytes = struct.pack('<H', id)
	payload = id_bytes + can_payload
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
	
	
	while(1):
		cur_time = time.time()
		if(cur_time - ts > 0.5):
			ts = cur_time

			can_payload = bytearray(8)
			can_payload[0] = ord('h')
			can_payload[1] = ord('e')
			can_payload[2] = ord('l')
			can_payload[3] = ord('l')
			can_payload[4] = ord('o')
			can_payload[5] = ord('?')

			payload = create_can_payload(1, can_payload)
			pld_stuffed = PPP_stuff(payload)
			print(pld_stuffed)

			server_socket.sendto(pld_stuffed, target_addr)

		try:
			pkt,source_addr = server_socket.recvfrom(512)
			print("From: "+source_addr[0]+":"+str(source_addr[1])+": "+str(pkt))
		except BlockingIOError:
			pass


