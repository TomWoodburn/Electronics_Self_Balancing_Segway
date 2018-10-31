'''
-------------------------------------------------------
Name: Milestone 1 - Bluetooth
Creator:  Group 8
Date:   March 2018
Revision:  ?
-------------------------------------------------------
Drive motor via BlueFruit UART Friend
-------------------------------------------------------
'''
import pyb
from pyb import Pin, Timer, ADC, UART
import time
from oled_938 import OLED_938	# Use OLED display driver

import micropython
micropython.alloc_emergency_exception_buf(100)

# Set up OLED display to prompt trigger press
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64, external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Group 8')
oled.draw_text(0, 10, 'Milestone1:Bluetooth')
oled.draw_text(0, 20, 'Press USR button')
oled.display()
print('Performing Milestone 1')
print('Waiting for button press')

# Set up trigger
trigger = pyb.Switch()		# Create trigger switch object
while not trigger():		# Wait for trigger press
	time.sleep(0.001)
while trigger():
	pass			# Wait for release
print('Button pressed - Running')
oled.draw_text(0,30, 'Running Milestone 1')
oled.display()

# --- Motor set up --- #
# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

# --- Bluetooth set up --- #
# Initialise UART communication
uart = UART(6)
uart.init(9600, bits=8, parity = None, stop = 2)

# Define motor controls
def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)
def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
def A_stop():
	A1.low()
	A2.low()
	motorA.pulse_width_percent(0)
def B_forward(value):
	B2.high()
	B1.low()
	motorB.pulse_width_percent(value)
def B_back(value):
	B1.high()
	B2.low()
	motorB.pulse_width_percent(value)
def B_stop():
	B1.low()
	B2.low()
	motorB.pulse_width_percent(0)

# Use keypad U and D keys to control speed
DEADZONE = 5
speed = 0

# --- Main program loop --- #
while True: 	# loop forever until CTRL-C
	speed = int((pot.read() - 2048) * 200 / 4096) # control speed with potentiometer

	while (uart.any()!=5):    # wait for 10 chars
		pass
	command = uart.read(5)
	if command[3] ==ord('1'):
		if command[2]==ord('7'):
			oled.draw_text(0, 40, 'Motor Drive:{:d}'.format(command[2]))
			A_forward(speed)
			B_forward(speed)
		elif command[2]==ord('8'):
			oled.draw_text(0, 40, 'Motor Drive:{:d}'.format(command[2]))
			A_back(speed)
			B_back(speed)
		elif command[2]==ord('6'):
			oled.draw_text(0, 40, 'Motor Drive:{:d}'.format(command[2]))
			A_forward(speed)
			B_back(speed)
		elif command[2] ==ord('5'):
			oled.draw_text(0, 40, 'Motor Drive:{:d}'.format(command[2]))
			A_back(speed)
			B_forward(speed)
	elif command[3] ==ord('0'):
		if command[2] ==ord('5') or command[2] ==ord('6') or command[2] ==ord('7') or command[2] ==ord('8'):
			oled.draw_text(0, 40, 'Motor Drive:{:d}'.format(command[3]))
			A_stop()
			B_stop()

	oled.display()
