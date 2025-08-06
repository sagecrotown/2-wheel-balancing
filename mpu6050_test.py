import smbus
import time
import mpu6050
import math

class MPU:

	def __init__(self, alpha, sensor):

		self.gx = 0.0
		self.gy = 0.0
		self.gz = 0.0

		self.ax = 0.0
		self.ay = 0.0
		self.az = 0.0

		self.gRoll = 0.0
		self.gPitch = 0.0

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.prevTime = time.time()

		#TODO: find correct scale factors
		self.gyroScaleFactor = 131.0
		self.accScaleFactor = 8192.0

		self.alpha = alpha
		self.sensor = sensor

	def getRawData(self):
		self.gx = self.sensor.get_gyro_data()['x']
		self.gy = self.sensor.get_gyro_data()['y']
		self.gz = self.sensor.get_gyro_data()['z']

		self.ax = self.sensor.get_accel_data()['x']
		self.ay = self.sensor.get_accel_data()['y']
		self.az = self.sensor.get_accel_data()['z']

	def processIMUdata(self):
		# update raw data
		self.getRawData()

		# convert to instantaneous deg/s
		self.gx /= self.gyroScaleFactor
		self.gy /= self.gyroScaleFactor
		self.gz /= self.gyroScaleFactor

		# convert to g force
		self.ax /= self.accScaleFactor
		self.ay /= self.accScaleFactor
		self.az /= self.accScaleFactor

	def compFilter(self):
		self.processIMUdata()

		dt = time.time() - self.prevTime
		self.prevTime = time.time()

		aPitch = math.degrees(math.atan2(self.ay, self.az))
		aRoll = math.degrees(math.atan2(self.ax, self.az))

		self.gPitch += self.gx * dt
		self.gRoll -= self.gy * dt
		self.yaw += self.gz * dt

		# comp filter
		self.roll = self.alpha * (self.roll - self.gy * dt) + (1 - self.alpha) * aRoll
		self.pitch = self.alpha * (self.pitch + self.gx * dt) + (1 - self.alpha) * aPitch

		print("R: ", round(self.roll, 3), "P: ", round(self.pitch, 3), "Y: ", round(self.yaw, 3))

def main():

	alpha = 0.92
	sensor = mpu6050.mpu6050(0x68)

	mpu = MPU(alpha, sensor)

	start = time.time()
#	while (time.time() < (start + 20)):
	while True:
		mpu.compFilter()

if __name__ == '__main__':
	main()




