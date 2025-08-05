#!/home/sage/code/venv/bin/python3

from evdev import InputDevice, categorize, ecodes
from gpiozero import Device, Motor
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory()

motorA = Motor(forward=27, backward=22)
motorB = Motor(forward=23, backward=24)

dev = InputDevice('/dev/input/event2')

hat_x = 0
hat_y = 0
x = 0
y = 0
vel = 1

def left(vel):
	motorA.forward(speed=vel)
	motorB.forward(speed=vel)

def right(vel):
	motorA.backward(speed=vel)
	motorB.backward(speed=vel)

def forward(vel):
	motorA.backward(speed=vel)
	motorB.forward(speed=vel)

def backward(vel):
	motorA.forward(speed=vel)
	motorB.backward(speed=vel)

def stop():
	motorA.stop()
	motorB.stop()

print("listening for controller input ... ")

for event in dev.read_loop():
	if event.type == ecodes.EV_ABS:

		if event.code == ecodes.ABS_HAT0X or event.code == ecodes.ABS_HAT0Y:

			if event.code == ecodes.ABS_HAT0X:
				hat_x = event.value
				print("x: ", hat_x)
			elif event.code == ecodes.ABS_HAT0Y:
				hat_y = event.value
				print("y: ", hat_y)

			if hat_y == -1:
				forward(vel)
			elif hat_y == 1:
				backward(vel)
			elif hat_x == -1:
				left(vel)
			elif hat_x == 1:
				right(vel)
			elif hat_x == 0 and hat_y == 0:
				stop()

		elif event.code == ecodes.ABS_X or event.code == ecodes.ABS_Y:

			if event.code == ecodes.ABS_X:
				x = (event.value/32767)
				#print("x: ", x)
			elif event.code == ecodes.ABS_Y:
				y = (event.value/32767)
				print("y: ", y)

			if y < 0:
				print("forward")
				forward(abs(y))
			elif y > 0:
				print("backward")
				backward(y)
			elif x < 0:
				left(abs(x))
			elif x > 0:
				right(x)
			elif x == 0 and y == 0:
				stop()
