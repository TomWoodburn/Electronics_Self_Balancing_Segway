'''
-------------------------------------------------------
Name: Milestone 3 - Balance
Creator:  Group 8
Date:   March 2018
Revision:  ?
-------------------------------------------------------
Balance segway via PID
-------------------------------------------------------
'''

import pyb
from pyb import Pin, Timer, ADC, DAC, LED, ExtInt
import time
from oled_938 import OLED_938
from mpu6050 import MPU6050
from motor import DRIVE

import micropython
micropython.alloc_emergency_exception_buf(100)

# Use OLED to say what segway is doing
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64, external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Group 8')
oled.draw_text(0, 10, 'Milestone3:Balance')
oled.draw_text(0, 20, 'Press USR button')
oled.display()

print('Performing Milestone 3')
print('Waiting for button press')
trigger = pyb.Switch()		# Create trigger switch object
while not trigger():		# Wait for trigger press
	time.sleep(0.001)
while trigger():
	pass			# Wait for release
print('Button pressed - Running')

# --- Balance set up --- #
# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

imu = MPU6050(1, False)

motor = DRIVE()

# PID tuning
scaleP = 10
scaleD = 1
scaleI = 100
while not trigger():	# Wait to tune Kp
	time.sleep(0.001)
#	K_p = pot.read() * scaleP / 4095		# Use potentiometer to set Kp
	K_p = 5.43
	oled.draw_text(0, 30, 'Kp={:5.2f}'.format(K_p))	# Display live value on oled
	oled.display()
while trigger(): pass

while not trigger():	# Wait to tune Kd
	time.sleep(0.001)
#	K_d = pot.read() * scaleD / 4095		# Use pot to set Ki
	K_d = 0.36
	oled.draw_text(60, 30, 'Kd={:5.2f}'.format(K_d))	# Display live value on oled
	oled.display()
while trigger(): pass

while not trigger():	# Wait to tune Ki
	time.sleep(0.001)
#	K_i = pot.read() * scaleI / 4095		# Use pot to set Ki
	K_i = 0.34
	oled.draw_text(0, 40, 'Ki={:5.2f}'.format(K_i))	# Display live value on oled
	oled.display()
while trigger(): pass

# Tuning Offset
scaleR = -3.6
while not trigger():	# Wait to tune Kp
	time.sleep(0.001)
#	r = pot.read() * scaleR / 4095		# Use potentiometer to set Kp
	r = -3.765
	oled.draw_text(60, 40, 'r={:5.3f}'.format(r))	# Display live value on oled
	oled.display()
while trigger(): pass


# Pitch angle calculation using complementary filter
def pitch_estimation(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt*0.000001) + (1-alpha)*theta
    print(pitch)
    #print("filtered = " + str(pitch))
    #print("imu = " + str(theta))
    return (pitch, pitch_dot)

alpha = 0.95
pitch = 0
e_int = 0
e_diff = 0
v = 0

# --- Main program loop --- #
tic = pyb.micros()

while True:
    dt = pyb.micros()-tic
    if dt > 5000:
        pitch, pitch_dot = pitch_estimation(pitch, dt, alpha)
        # PID control
        e = pitch - r
        v = (K_p*e + K_i*e_int + K_d*pitch_dot)
        e_int += e

        if v > 0:
            motor.right_forward(v)
            motor.left_forward(v)
        if v < 0:
            motor.right_back(v)
            motor.left_back(v)

        e_diff = e

        tic = pyb.micros()
