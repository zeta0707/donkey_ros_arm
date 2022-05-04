#!/usr/bin/python
from time import sleep, time
import myconfig as mc
import myutil as mu

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

pwm.set_pwm(0, 1,  mc.MOTOR1_HOME)       
pwm.set_pwm(0, 2,  mc.MOTOR2_HOME)  
pwm.set_pwm(0, 3,  mc.MOTOR3_HOME) 
pwm.set_pwm(0, 14, mc.MOTOR4_HOME)
pwm.set_pwm(0, 15, mc.GRIPPER_HOME)
pwm.set_pwm(0, 0,  mc.MOTOR0_HOME) 

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
    try:
        sleep(float(time_diff))
    except KeyboardInterrupt:
       break
    #print(str(motor0)+':'+str(motor1)+':'+str(motor2)+':'+str(motor3)+':'+str(motor4)+':'+str(motor5)+':'+str(time_diff))

print("automove reached end")
file1.close()
pwm.set_pwm(0, 14, mc.MOTOR4_OFF)
pwm.set_pwm(0, 15, mc.GRIPPER_OFF)
pwm.set_pwm(0, 0,  mc.MOTOR0_OFF) 
sleep(2)
pwm.set_all_pwm(0,0)
