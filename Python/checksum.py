import struct
import numpy as np


def fletchers_checksum16(bytes_in):
	num_words = int(len(bytes_in)/2)
	fmt_str = '<'+'H'*num_words
	#ui16 = struct.unpack(fmt_str, bytes_in)
	i16 = struct.unpack(fmt_str, bytes_in)
	chksum = 0
	fchk = 0
	for val in i16:
		chksum = (chksum + val) & 0xFFFF
		fchk = (fchk + chksum) & 0xFFFF
	return fchk
