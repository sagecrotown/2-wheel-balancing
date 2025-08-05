import RPi.GPIO as GPIO
import time

def setup():
	GPIO.setmode(GPIO.BCM)

	global r_pin, g_pin, b_pin
	global r_pwm, g_pwm, b_pwm
	global color_list

	r_pin = 18
	g_pin = 17
	b_pin = 27

	GPIO.setup(r_pin, GPIO.OUT)
	GPIO.setup(g_pin, GPIO.OUT)
	GPIO.setup(b_pin, GPIO.OUT)

	freq = 500

	r_pwm = GPIO.PWM(r_pin, freq)
	g_pwm = GPIO.PWM(g_pin, freq)
	b_pwm = GPIO.PWM(b_pin, freq)

	r_pwm.start(0)
	g_pwm.start(0)
	b_pwm.start(0)

	color_list = [[0, 0, 0], [255, 255, 255], [255, 0, 0], [0, 255, 0], [0, 0, 255]]

# map takes values in the 0-255 range and maps them to 0-100
def map(x):
	if (x>255) or (x<0):
		print('ERROR: failed to provide appropriate RGB value')
		destroy()
	else:
		return (x / 255 * 100)

# set_color takes RGB values in the 0-255 range, translates to PWM (0-100), and sets it
def set_color(color):
	r = color[0]
	g = color[1]
	b = color[2]

	r = map(r)
	g = map(g)
	b = map(b)

	r_pwm.ChangeDutyCycle(r)
	g_pwm.ChangeDutyCycle(g)
	b_pwm.ChangeDutyCycle(b)

# stop everything
def destroy():
	global r_pwm, g_pwm, b_pwm
	r_pwm.stop()
	g_pwm.stop()
	b_pwm.stop()
	del r_pwm, g_pwm, b_pwm
	GPIO.cleanup()	# release resource

def main():
	while True:
		for color in color_list:
			set_color(color)
			time.sleep(0.5)

if __name__ == '__main__':
	setup()
	try:
		main()
	except KeyboardInterrupt:
		destroy()
