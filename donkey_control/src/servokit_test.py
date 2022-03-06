#!/usr/bin/python

from time import sleep

# Import the PCA9685 module.
import Adafruit_PCA9685
from Adafruit_GPIO import I2C

def get_bus():
    return 1

I2C.get_default_bus = get_bus

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths
servo_min = 270  # Min pulse length out of 4096
servo_max = 490  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
channel = int(input('Channel 0~15: '))

while 1:
    pulse = raw_input("Insert position:")
    if pulse=="A":
        for i in range(servo_min,servo_max):
            pwm.set_pwm(channel,0,i)
            sleep(0.001)
        for i in range(servo_max,servo_min,-1):
            pwm.set_pwm(channel,0,i)
            sleep(0.001)
    else:
        i=int(pulse)
        pwm.set_pwm(channel,0,i)
