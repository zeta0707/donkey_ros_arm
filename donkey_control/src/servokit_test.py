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

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
channel = 0

while 1:
    user_input = raw_input('Channel 0~15: ')
    if user_input:
        channel = int(user_input)
        
    pulse = raw_input("Insert position or q:")
    if pulse=="q":
        break
    else:
        i=int(pulse)
        pwm.set_pwm(channel,0,i)

pwm.set_all_pwm(0,0)