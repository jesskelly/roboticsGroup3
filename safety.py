#! /usr/bin/env python3
# RUN ON PYTHON3!!!!!!!!!!!!!!!!!!!!!!!
import time
import board
import busio
import adafruit_adxl34x
import numpy as np
import math
from time import process_time

def MotorOn (currentState, acceleration, t0, start_time):
    timeSleollep = 0.2
    threshold = -7

    while currentState == 1:
        acceleration[0,:] = acceleration[1,:]
        t1 = t0
        t0 = process_time()
        timeStep = t0-t1+ timeSleep
        #print(timeStep)
        acceleration[1,:] = np.asarray(accelerometer.acceleration)
        #print("current on")

        # Take the derivative
        derivative = (math.sqrt(acceleration[1,0]**2+acceleration[1,1]**2+acceleration[1,2]**2) - math.sqrt(acceleration[0,0]**2+acceleration[0,1]**2+acceleration[0,2]**2))/timeStep
        print("Time: %f"%(time.time() - start_time))
        accelerometer.enable_tap_detection(tap_count=1, threshold=20, duration=50, latency=20, window=255)
        print("Tapped: %s"%accelerometer.events['tap'])
        #f.write("Tapped: %s \n"%accelerometer.events['tap'])
        print("Derivative is: %f"%derivative)

        if (derivative < threshold)  :
            flag = 1
            print('Collision detected')
            currentState = 0;  #actually cut the current

        time.sleep(timeSleep)

    MotorOff(currentState, acceleration, t0, start_time)

    return;

def MotorOff (currentState, acceleration, t0, start_time):
    while currentState == 0:
        currentState = 1

    MotorOn(currentState, acceleration, t0, start_time)
    return;

# Set Up
f = open("data.txt", "ab")
start_time = time.time()
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)
acceleration = np.zeros((2,3))
currentState = 1;
t0 = process_time()
#Check if current is on
if currentState == 1:
    MotorOn(currentState, acceleration, t0, start_time);
else:
    MotorOff(currentState, acceleration, t0, start_time);
    #Call either motorOn or motor Off