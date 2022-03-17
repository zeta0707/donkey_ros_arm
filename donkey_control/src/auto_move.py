#!/usr/bin/python
from time import sleep, time

# Import the PCA9685 module.
import Adafruit_PCA9685
from Adafruit_GPIO import I2C

def get_bus():
    return 1

I2C.get_default_bus = get_bus

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

file1 = open('automove.txt', 'r')
# Using readline() 

while True:
    # Get next line from file
    line = file1.readline()
    # if line is empty, end of file is reached
    if not line:
        break

    motor0, motor1, motor2, motor3, motor4, motor5, time_diff = line.split(':')
    pwm.set_pwm(0, 0, int(motor0))
    pwm.set_pwm(1, 0, int(motor1))
    pwm.set_pwm(2, 0, int(motor2))
    pwm.set_pwm(3, 0, int(motor3))
    pwm.set_pwm(14,0, int(motor4))
    pwm.set_pwm(15,0, int(motor5))
    sleep(float(time_diff))

file1.close()
