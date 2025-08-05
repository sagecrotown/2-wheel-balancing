import RPi.GPIO as GPIO
import time
LedPin = 18

def setup():
	# set GPIO modes to BCM numbering
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LedPin, GPIO.OUT, initial=GPIO.LOW)

def main():
	while True:
		print('LED OFF')
		GPIO.output(LedPin, GPIO.LOW)
		time.sleep(0.25)
		print('RED ON')
		GPIO.output(LedPin, GPIO.HIGH)
		time.sleep(1)

def destroy():
	# turn off LED
	GPIO.output(LedPin, GPIO.LOW)
	GPIO.cleanup()

if __name__ == '__main__':
	setup()
	try:
		main()
	except KeyboardInterrupt:
		destroy()

