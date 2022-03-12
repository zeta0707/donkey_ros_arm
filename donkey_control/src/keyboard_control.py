#!/usr/bin/env python

"""
Node for control PCA9685 using teleop_twist_keyboard msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

import time
import rospy
from threading import Thread
from geometry_msgs.msg import Twist

import myconfig as mc
import myutil as mu

yaw_pulse = mc.MOTOR0_HOME
pitch_pulse = mc.MOTOR4_HOME
gripper_pulse = mc.GRIPPER_OPEN    

roll_pulse = 0
roll_pulse1 = mc.MOTOR1_HOME
roll_pulse2 = mc.MOTOR2_HOME
roll_pulse3 = mc.MOTOR3_HOME

class RobotArm(object):
    def __init__(self, name="donkey_arm"):
        
        self.motor0 = mu.PCA9685(channel=0, address=0x40, busnum=1)
        self.motor1 = mu.PCA9685(channel=1, address=0x40, busnum=1)
        self.motor2 = mu.PCA9685(channel=2, address=0x40, busnum=1)
        self.motor3 = mu.PCA9685(channel=3, address=0x40, busnum=1)
        self.motor4 = mu.PCA9685(channel=14, address=0x40, busnum=1)
        self.motor5 = mu.PCA9685(channel=15, address=0x40, busnum=1)
        rospy.loginfo("PCA9685 Awaked!!")
        
        self._name = name
        self._teleop_sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.keyboard_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.motor0.run(mc.MOTOR0_ZERO)
        self.motor1.run(mc.MOTOR1_ZERO)
        self.motor2.run(mc.MOTOR2_ZERO)
        self.motor3.run(mc.MOTOR3_ZERO)           
        self.motor4.run(mc.MOTOR4_ZERO)  
        self.motor5.run(mc.GRIPPER_OPEN)  

        rospy.loginfo("Keyboard Subscriber Awaked!! Waiting for keyboard...")

    def keyboard_callback(self, msg):
        global yaw_pulse
        global roll_pulse  
        global roll_pulse1
        global roll_pulse2
        global pitch_pulse
        global gripper_pulse

        self.pulse_rem = 0

        #rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        
        if msg.linear.x == 0.0:
            yaw_pulse -= int(msg.angular.z)
        elif msg.angular.z == 0.0:
            roll_pulse += int(msg.linear.x*2.0)
        else:
            pitch_pulse += int(msg.linear.x*2.0)
        gripper_pulse += int(msg.linear.z*2.0)

        yaw_pulse = mu.clamp(yaw_pulse, mc.YAW_MIN,mc.YAW_MAX)
        pitch_pulse = mu.clamp(pitch_pulse, mc.PITCH_MIN, mc.PITCH_MAX)
        gripper_pulse = mu.clamp(gripper_pulse, mc.GRIPPER_MIN, mc.GRIPPER_MAX)

        self.pulse_rem, roll_pulse3 = mu.clampRem(roll_pulse, mc.MOTOR3_DIF_MIN, mc.MOTOR3_DIF_MAX)
        self.pulse_rem, roll_pulse2 = mu.clampRem(self.pulse_rem, mc.MOTOR2_DIF_MIN, mc.MOTOR2_DIF_MAX)
        self.pulse_rem, roll_pulse1 = mu.clampRem(self.pulse_rem, mc.MOTOR1_DIF_MIN, mc.MOTOR1_DIF_MAX)

        roll_pulse3 += mc.MOTOR3_HOME
        roll_pulse2 += mc.MOTOR2_HOME
        roll_pulse1 += mc.MOTOR1_HOME

        print(
            "motor0_pulse : "
            + str(yaw_pulse)
            + " / "
            + "roll_pulse : "
            + str(roll_pulse)
            + " / "
            + "motor1_pulse : "
            + str(roll_pulse1)
            + " / "
            + "motor2_pulse : "
            + str(roll_pulse2)  
            + " / "
            + "motor3_pulse : "
            + str(roll_pulse3)        
        )

        self.motor0.run(yaw_pulse)       #control by joystick
        self.motor1.run(roll_pulse1)     #control by joystick
        self.motor2.run(roll_pulse2)     #control by joystick
        self.motor3.run(roll_pulse3)     #control by joystick
        self.motor4.run(pitch_pulse)     #control by joystick
        self.motor5.run(gripper_pulse)    #control by joystick

if __name__ == "__main__":

    rospy.init_node("donkey_control")
    myArm = RobotArm("donkey_ros")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
