#! /usr/bin/env python3
# RUN ON PYTHON3!!!!!!!!!!!!!!!!!!!!!!!
import adafruit_adxl34x
import board
import busio
import MotorState
import math
import numpy as np
import time


def CurrentSensor():
    reading = 1  # get actual current sensor reading
    return reading


def Accelerometer():
    i2c = busio.I2C(board.SCL, board.SDA)
    accelerometer = adafruit_adxl34x.ADXL345(i2c)
    return accelerometer


def CutCurrent():
    # cut the current
    currentState = 0
    return currentState


def MotorOff(f, accelerometer, timeStep, threshold):
    print("Motor is off")  # do nothing
    f.write("Motor is off")
    return


def CollisionDetection(f, accelerometer, timeStep, threshold):

    acceleration = np.zeros((2, 3))  # Initialize

    while(True):
        acceleration[0, :] = acceleration[1, :]
        acceleration[1, :] = np.asarray(accelerometer.acceleration)

        # Take the derivative - norm of 3 directions
        derivative = (math.sqrt(acceleration[1, 0]**2+acceleration[1, 1]**2
                                + acceleration[1, 2]**2) - math.sqrt(
                                acceleration[0, 0]**2 + acceleration[0, 1]**2
                                + acceleration[0, 2]**2))/timeStep

        # Trying with tap detection
        accelerometer.enable_tap_detection(tap_count=1, threshold=20,
                                           duration=50, latency=20,
                                           window=255)

        print("Tapped: %s" % accelerometer.events['tap'])  # True or False
        f.write("Tapped: %s \n" % accelerometer.events['tap'])
        print("Derivative is: %f" % derivative)

        if (derivative < threshold):
            print('Collision detected')
            CutCurrent()
            # cut the current

            time.sleep(timeStep)  # difference between measurements

    return


def Main():
    timeStep = 0.2  # define how frequently measurements are read.
    threshold = -7
    filename = "trial1.txt"

    # CollectData prints the output to a file with name filename
    f = open(filename, "ab")

    # Set-up connections
    accelerometer = Accelerometer()
    currentSensor = CurrentSensor()

    # Collision Detection
    while(True):
        if currentSensor != 0:
            MotorState.CollisionDetection(f, accelerometer, timeStep,
                                          threshold)
        else:  # do nothing
            MotorOff()
    return


# Run the main program
Main()
