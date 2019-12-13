#! /usr/bin/python3
# RUN ON PYTHON3!!!!!!!!!!!!!!!!!!!!!!!
import time
import board
import busio
import adafruit_adxl34x
import numpy as np
from time import process_time

def MotorOn (currentState, acceleration, t0):
    timeSleep = 2
    while True:
        acceleration[0,:] = acceleration[1,:]
        t1 = t0
        t0 = process_time()
        timeStep = t0-t1+ timeSleep
        print(timeStep)
        acceleration[1,:] = np.asarray(accelerometer.acceleration)
        print("current on")
        print(acceleration)

        # Take the derivative
        derivative = (acceleration[1,:] - acceleration[0,:])/timeStep
        print(derivative)

        if currentState == 0:
            break;
        time.sleep(timeSleep)

    MotorOff(currentState, acceleration, t0)
    return;

def MotorOff (currentState, acceleration, t0):
    print("current off")
    return;

# Set Up
f = open("data.txt", "w+")
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)
acceleration = np.zeros((2,3))
currentState = 1;
t0 = process_time()
#Check if current is on
if currentState == 1:
    MotorOn(currentState, acceleration, t0);
else:
    MotorOff(currentState, acceleration, t0);
    #Call either motorOn or motor Off


# while True:
#     print("%f %f %f"%accelerometer.acceleration)
#     f.write("%f %f %f \n"%accelerometer.acceleration)

#     accelerometer.enable_tap_detection(tap_count=1, threshold=20, duration=50, latency=20, window=255)
#     print("Tapped: %s"%accelerometer.events['tap'])
#     f.write("Tapped: %s \n"%accelerometer.events['tap'])

#     time.sleep(1)