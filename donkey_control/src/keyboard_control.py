#!/usr/bin/env python

"""
Node for control PCA9685 using teleop_twist_keyboard msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

from time import time, sleep
import rospy
from threading import Thread
from geometry_msgs.msg import Twist

import myconfig as mc
import myutil as mu

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
        rospy.loginfo("Keyboard Subscriber Awaked!! Waiting for keyboard...")
        self.done_home = 0

        # start from off position
        self.motor2.run(mc.MOTOR2_OFF)
        self.motor3.run(mc.MOTOR3_OFF)       
        self.motor4.run(mc.MOTOR4_OFF)  
        self.motor1.run(mc.MOTOR1_OFF) 
        self.motor5.run(mc.GRIPPER_OFF) 
        self.motor0.run(mc.MOTOR0_OFF)

        # move to home positon from off
        self.motor1.runTarget(mc.MOTOR1_OFF, mc.MOTOR1_HOME)
        self.motor2.runTarget(mc.MOTOR2_OFF, mc.MOTOR2_HOME)
        self.motor3.runTarget(mc.MOTOR3_OFF, mc.MOTOR3_HOME)    
        self.motor4.runTarget(mc.MOTOR4_OFF, mc.MOTOR4_HOME)  
        self.motor5.runTarget(mc.GRIPPER_OFF, mc.GRIPPER_HOME) 
        self.motor0.runTarget(mc.MOTOR0_OFF, mc.MOTOR0_HOME)     

        self.pulse_rem = 0
        self.done_home = 1
        self.fhandle = open("automove.txt", 'w')

        self.yaw_pulse = mc.MOTOR0_HOME
        self.pitch_pulse = mc.MOTOR4_HOME
        self.gripper_pulse = mc.GRIPPER_HOME 
        self.roll_pulse = 0
        self.roll_pulse1 = mc.MOTOR1_HOME
        self.roll_pulse2 = mc.MOTOR2_HOME
        self.roll_pulse3 = mc.MOTOR3_HOME 

    def __del__(self):
        print("Arm class release")
        self.motor5.run(mc.GRIPPER_OFF) 
        self.motor4.run(mc.MOTOR4_OFF) 
        self.motor0.run(mc.MOTOR0_OFF)
        self.motor0.set_pwm_clear()
        self.fhandle.close

    def keyboard_callback(self, msg):
        if  self.done_home == 0:
            return
        else:
            if self.done_home == 1:                
                self.done_home = 2
                self.prev_time = time()

        self.pulse_rem = 0

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        
        if msg.linear.x == 0.0:
            self.yaw_pulse -= int(msg.angular.z*3.0)
        elif msg.angular.z == 0.0:
            self.roll_pulse += int(msg.linear.x*3.0)
        else:
            self.pitch_pulse += int(msg.linear.x*10.0)
        self.gripper_pulse += int(msg.linear.z*10.0)

        #rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        self.yaw_pulse = mu.clamp(self.yaw_pulse, mc.YAW_MIN,mc.YAW_MAX)
        self.pitch_pulse = mu.clamp(self.pitch_pulse, mc.PITCH_MIN, mc.PITCH_MAX)
        self.gripper_pulse = mu.clamp(self.gripper_pulse, mc.GRIPPER_MIN, mc.GRIPPER_MAX)
        self.roll_pulse = mu.clamp(self.roll_pulse, mc.ROLL_TOTAL_MIN, mc.ROLL_TOTAL_MAX)

        self.pulse_rem, self.roll_pulse3 = mu.clampRem(self.roll_pulse, mc.MOTOR3_DIF_MIN, mc.MOTOR3_DIF_MAX)
        self.pulse_rem, self.roll_pulse2 = mu.clampRem(self.pulse_rem, mc.MOTOR2_DIF_MIN, mc.MOTOR2_DIF_MAX)
        self.pulse_rem, self.roll_pulse1 = mu.clampRem(self.pulse_rem, mc.MOTOR1_DIF_MIN, mc.MOTOR1_DIF_MAX)

        self.roll_pulse3 += mc.MOTOR3_HOME
        self.roll_pulse2 += mc.MOTOR2_HOME
        self.roll_pulse1 += mc.MOTOR1_HOME

        print(
            "motor0_pulse : "
            + str(self.yaw_pulse)
            + " / "
            + "roll_pulse : "
            + str(self.roll_pulse)
            + " / "
            + "motor1_pulse : "
            + str(self.roll_pulse1)
            + " / "
            + "motor2_pulse : "
            + str(self.roll_pulse2)  
            + " / "
            + "motor3_pulse : "
            + str(self.roll_pulse3)        
        )

        self.motor0.run(self.yaw_pulse)       #control by joystick
        self.motor1.run(self.roll_pulse1)     #control by joystick
        self.motor2.run(self.roll_pulse2)     #control by joystick
        self.motor3.run(self.roll_pulse3)     #control by joystick
        self.motor4.run(self.pitch_pulse)     #control by joystick
        self.motor5.run(self.gripper_pulse)   #control by joystick
        
        self.timediff = time() - self.prev_time
        self.prev_time = time()
        self.fhandle.write(str(self.yaw_pulse) + ':' + str(self.roll_pulse1) + ':' + str(self.roll_pulse2) 
        + ':' + str(self.roll_pulse3) + ':' + str(self.pitch_pulse) + ':' + str(self.gripper_pulse) + ':' + str(self.timediff) + '\n')

if __name__ == "__main__":

    rospy.init_node("donkey_control")
    myArm = RobotArm("donkey_ros")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    myArm.__del__()
