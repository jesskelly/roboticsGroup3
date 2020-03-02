#! /usr/bin/env python3
# RUN ON PYTHON3!!!!!!!!!!!!!!!!!!!!!!!


import adafruit_adxl34x
import board
import busio
import math
import numpy as np
import serial
import time


def CollisionDetection():

    timeStep = 0.5  # define how frequently measurements are read.
    threshold = -7
    acceleration = np.zeros((2, 3))  # Initialize
    
    # Set-up accelerometer connections, using an i2c communication.
    i2c = busio.I2C(board.SCL, board.SDA)
    accelerometer = adafruit_adxl34x.ADXL345(i2c)

    # Collision detection while motor is on
    while True:
        
        # Set-up current sensor connection, using serial communication.
        ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
        ser.flush()
        
        # Get current sensor reading.
        if ser.in_waiting > 0:
            currentVal = ser.readline().decode('utf-8').strip()
            print('The current sensor value is: %f' % float(currentVal))

            if float(currentVal) > 2:
                print('Motor on \n')
                
                # Set-up acceleration matrix
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

                print("Tapped: %s \n" % accelerometer.events['tap'])  # True or False
                print("Derivative is: %f \n" % derivative)

                # Detect collision:
                if (derivative < threshold):
                    print('COLLISION DETECTED')
                
            else:  # do nothing
                print("Motor is off, raise a flag")  # do nothing
            
        time.sleep(timeStep)  # difference between measurements

    return


def Main():
    
    CollisionDetection()
    
    return


# Run the main program
Main()