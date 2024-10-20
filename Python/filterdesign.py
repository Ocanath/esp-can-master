"""
	Filter design experimentation with the scipy utility.
	
	NOTE: iirfilter produces EXACTLY the same results as fdatool. Tested with the following parameters:
		IIR
		butterworth
		specify order: 2
		Fs: 30 (same for both)
		Wn/Fc: .1 (same for both)
		
	Only difference is that matlab does not apply the gain in the numerator coefficients,
	while scipy does apply the gain in the numerator coefficients.
"""

from scipy import signal
import matplotlib.pyplot as plt
import matplotlib.ticker
import numpy as np


format = 'not embedded'

# Fs= 1000 #radians per sample
# wn = 100	#rad/s?


#system = signal.iirdesign(wp, ws, gpass, gstop, analog=False, ftype='butter', output='sos', fs=Fs)
system = signal.iirfilter(2, 0.02, btype='lowpass', analog=False, ftype='bessel', output='sos')
#print(len(system))
#print (system)
sos = system[0]
b_mag = np.sqrt(np.dot(sos[0:3],sos[0:3]))
if(format == 'embedded'):
	print("Raw:", sos, "\n\n")
	print("coef_float = {")
	print("    .a0 = ",sos[3], ",", "//(usually 1)")
	print("    .a1 = ",sos[4], ",")
	print("    .a2 = ",sos[5], ",")
	print("    .b0 = ",sos[0]/b_mag, ",")
	print("    .b1 = ",sos[1]/b_mag, ",")
	print("    .b2 = ",sos[2]/b_mag, ",")
	print("    .gain = ",b_mag)
	print("};\n")
else:
	print("Raw:", sos, "\n\n")
	print("coef_float = {")
	print("    ",sos[4], ", //a1")
	print("    ",sos[5], ",//a2")
	print("    ",sos[0]/b_mag, ", //b0")
	print("    ",sos[1]/b_mag, ", //b1")
	print("    ",sos[2]/b_mag, ", //b2")
	print("    ",b_mag, ", //gain")
	print("    {0, 0, 0}")
	print("};\n")


#manual settings for digital filter implementation: radix
radix_num = 18
radix_den = 18
print("iirsos_t filt = \n{")
print("    .a0 = ",np.floor(sos[3]*2**radix_num).astype(np.int32),", //(usually", 2**radix_num, ")")
print("    .a1 = ",np.floor(sos[4]*2**radix_num).astype(np.int32),",")
print("    .a2 = ",np.floor(sos[5]*2**radix_num).astype(np.int32),",")
print("    .radix_num = ", radix_num,",") 
print("    .b0 = ",np.floor(sos[0]*2**radix_den).astype(np.int32),",")
print("    .b1 = ",np.floor(sos[1]*2**radix_den).astype(np.int32),",")
print("    .b2 = ",np.floor(sos[2]*2**radix_den).astype(np.int32),",")
print("    .radix_den = ", radix_den)
print("};\n");




w,h = signal.sosfreqz(system, worN=512, whole=False)

fig, ax1 = plt.subplots()
ax1.set_title('Digital filter frequency response')
ax1.plot(w, 0*abs(h), 'r')
ax1.plot(w, 20 * np.log10(abs(h)), 'b')
ax1.set_ylabel('Amplitude [dB]', color='b')
ax1.set_xlabel('Frequency [rad/sample]')
ax1.grid()
#ax1.set_ylim([-120, 20])
ax1.relim()
ax1.autoscale_view()
ax2 = ax1.twinx()
angles = np.unwrap(np.angle(h)*180/np.pi)
ax2.plot(w, angles, 'g')
ax2.set_ylabel('Phase (degrees)', color='g')
ax2.grid()
#ax2.axis('tight')
ax2.relim()
ax2.autoscale_view()
nticks = 8
ax1.yaxis.set_major_locator(matplotlib.ticker.LinearLocator(nticks))
ax2.yaxis.set_major_locator(matplotlib.ticker.LinearLocator(nticks))
plt.show()