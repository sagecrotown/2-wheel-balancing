#!/home/sage/code/venv/bin/python3

from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory()

from gpiozero import Motor
import time
import sys
import tty
import termios
import select

# define motor pins
motorA = Motor(forward = 27, backward = 22)
motorB = Motor(forward = 23, backward = 24)

# save terminal settings
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

# keyboard processing
def get_key():
        tty.setraw(fd)  # set raw mode
        # Check if input is ready (timeout 0 = non-blocking)
        dr, _, _ = select.select([sys.stdin], [], [], 0.05)
        if dr:
                ch = sys.stdin.read(1)
        else:
                ch = 0 # no key pressed
        return ch

# time loop lol
while True:
        key = get_key()
        print(key)
        vel=1
        if (key == 'a'):
                motorA.forward(speed=vel)
                motorB.forward(speed=vel)
        elif (key == 's'):
                motorA.forward(speed=vel)
                motorB.backward(speed=vel)
        elif (key == 'd'):
                motorA.backward(speed=vel)
                motorB.backward(speed=vel)
        elif (key == 'w'):
                motorA.backward(speed=vel)
                motorB.forward(speed=vel)
        elif(key == 0):
                motorA.stop()
                motorB.stop()
        else:
                break

# stop motors and fix terminal settings
motorA.stop()
motorB.stop()
termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
