#!/home/sage/code/venv/bin/python3

from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory()

from gpiozero import Motor
import time

# define motor pins
motorA = Motor(forward = 20, backward = 21)
motorB = Motor(forward = 5, backward = 6)
vel = 1

# drive forward, then backward, at full speed
motorA.forward(speed=vel)
motorB.backward(speed=vel)
time.sleep(2)

motorA.backward(speed=vel)
motorB.forward(speed=vel)
time.sleep(2)

# half speed
vel = .5

motorA.forward(speed=vel)
motorB.backward(speed=vel)
time.sleep(2)

motorA.backward(speed=vel)
motorB.forward(speed=vel)
time.sleep(2)

# stop motors
motorA.stop()
motorB.stop()
