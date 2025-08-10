#!/home/sage/code/venv/bin/python3

import smbus
import time
import mpu6050
import math

from evdev import InputDevice, categorize, ecodes
from gpiozero import Device, Motor
from gpiozero.pins.pigpio import PiGPIOFactory

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

                # print("R: ", round(self.roll, 3), "P: ", round(self.pitch, 3), "Y: ", round(self.yaw, 3))

        def getRoll(self):
                self.compFilter()
                return self.roll

        def getPitch(self):
                self.compFilter()
                return self.pitch
        
        def getYaw(self):
                self.compFilter()
                return self.yaw

class Balancing:

        def __init__(self, aForward, aBackward, bForward, bBackward, KP, KI, KD, inputDev, MPU):
                self.motorA = Motor(forward=aForward, backward=aBackward)
                self.motorB = Motor(forward=bForward, backward=bBackward)

                self.dev = InputDevice(inputDev)
                self.mpu = MPU

                self.kp = KP
                self.ki = KI
                self.kd = KD
                self.prevErr = 0
                self.sumErr = 0
                self.prevTime = time.time()

                self.hat_x = 0
                self.hat_y = 0
                self.x = 0
                self.y = 0
                self.vel = 1

        def balance(self, targetAngle = 0):
                time = time.time()
                error = self.mpu.getPitch() - targetAngle
                proportional = self.kp * error
                integral += self.ki *self.error
                derivative = self.kd * (error - self.prevErr) / (time - self.prevTime)
                self.prevTime = time
                self.prevErr = error
                effort = proportional + integral + derivative

                print("angle: ", round(error, 2), "effort: ", round(effort, 2))

                return effort
        
        def drive(self, goalSpeed = 0):
                balanceEffort = self.balance()
                if (balanceEffort + goalSpeed > 1):
                        self.motorA.forward(1)
                        self.motorB.backward(1)
                elif (balanceEffort - goalSpeed < -1):
                        self.motorA.backward(1)
                        self.motorB.forward(1)
                elif (balanceEffort + goalSpeed > 0):
                        self.motorA.forward(balanceEffort + goalSpeed)
                        self.motorB.backward(balanceEffort + goalSpeed)
                elif (balanceEffort - goalSpeed < 0):
                        self.motorA.backward(-balanceEffort + goalSpeed)
                        self.motorB.forward(-balanceEffort + goalSpeed)
                

def main():

        alpha = 0.92
        sensor = mpu6050.mpu6050(0x68)

        mpu = MPU(alpha, sensor)

        aFor = 27
        aBack = 22
        bFor = 23
        bBack = 24

        kp = 1
        ki = 0
        kd = 0

        dev = InputDevice('/dev/input/event2') 

        robot = Balancing(aFor, aBack, bFor, bBack, kp, ki, kd, dev, mpu)

        start = time.time()
#       while (time.time() < (start + 20)):
        while True:
                # mpu.compFilter()
                robot.drive()

if __name__ == '__main__':
        main()

class PID:

	def __init__(self, kp, ki, kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd

		self.prevTime = time.time()

