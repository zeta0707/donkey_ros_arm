#!/usr/bin/env python

"""
Node for control PCA9685 using teleop_twist_keyboard msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

from time import time, sleep
import rospy
from threading import Thread
from sensor_msgs.msg import JointState

import myconfig as mc
from myutil import clamp, clampRem, PCA9685, trimLimits, radiansToDegrees

class RobotArm(object):
    def __init__(self, name="donkey_arm"):
        
        self.motor0 = PCA9685(channel=0, address=0x40, busnum=1)
        self.motor1 = PCA9685(channel=1, address=0x40, busnum=1)
        self.motor2 = PCA9685(channel=2, address=0x40, busnum=1)
        self.motor3 = PCA9685(channel=3, address=0x40, busnum=1)
        self.motor4 = PCA9685(channel=14, address=0x40, busnum=1)
        self.motor5 = PCA9685(channel=15, address=0x40, busnum=1)
        rospy.loginfo("PCA9685 Awaked!!")

        self._name = name
        self._teleop_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.moveit_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.loginfo("Moveit Subscriber Awaked!! Waiting for Moveit Planning...")
        self.armStatus = "Homing"

        # start from off position
        self.goPosOff()

        # move to home positon from off
        self.motor1.runTarget(mc.MOTOR1_OFF, mc.MOTOR1_HOME)
        self.motor2.runTarget(mc.MOTOR2_OFF, mc.MOTOR2_HOME)
        self.motor3.runTarget(mc.MOTOR3_OFF, mc.MOTOR3_HOME)    
        self.motor4.runTarget(mc.MOTOR4_OFF, mc.MOTOR4_HOME)  
        self.motor5.runTarget(mc.GRIPPER_OFF, mc.GRIPPER_HOME) 
        self.motor0.runTarget(mc.MOTOR0_OFF, mc.MOTOR0_HOME)     

        self.armStatus = "DoneHoming"

        self.yaw_pulse = mc.MOTOR0_HOME
        self.pitch_pulse = mc.MOTOR4_HOME
        self.gripper_pulse = mc.GRIPPER_HOME 
        self.roll_pulse1 = mc.MOTOR1_HOME
        self.roll_pulse2 = mc.MOTOR2_HOME
        self.roll_pulse3 = mc.MOTOR3_HOME 

    def __del__(self):
        print("Arm class release")
        self.goPosOff()
        sleep(1)        
        self.motor0.set_pwm_clear()

    def goPosOff(self):
        self.motor2.run(mc.MOTOR2_OFF)
        self.motor3.run(mc.MOTOR3_OFF)       
        self.motor4.run(mc.MOTOR4_OFF)  
        self.motor1.run(mc.MOTOR1_OFF) 
        self.motor5.run(mc.GRIPPER_OFF) 
        self.motor0.run(mc.MOTOR0_OFF)

    def translate(self, sensor_val, in_from, in_to, out_from, out_to):
        out_range = out_to - out_from
        in_range = in_to - in_from
        in_val = sensor_val - in_from
        val=(float(in_val)/in_range)*out_range
        out_val = out_from+val
        return out_val

   # Function to move motor to specific position
    def DegToPulse(self, moveDegree):
        # Convert to pulse width
        pulse_wide = self.translate(moveDegree, 0, 180, mc.MIN_PULSE_WIDTH, mc.MAX_PULSE_WIDTH)
        return pulse_wide

    def moveit_callback(self, cmd_msg):
        if  self.armStatus == "Homing":
            return
        else:
            if self.armStatus == "DoneHoming":                
                self.armStatus = "MoveitControled"
                self.prev_time = time()

        #print( str(cmd_msg.position[0]) + ':' + str(cmd_msg.position[1]) + ':' + str(cmd_msg.position[2]) 
        #+ ':' + str(cmd_msg.position[3]) + ':' + str(cmd_msg.position[4]) )

        tempDegree = trimLimits(radiansToDegrees(cmd_msg.position[0]) + 90)
        self.yaw_pulse = int(self.DegToPulse(tempDegree))
        tempDegree = trimLimits(radiansToDegrees(cmd_msg.position[1]) + 90)
        self.roll_pulse1 = int(self.DegToPulse(tempDegree))
        tempDegree = trimLimits(radiansToDegrees(cmd_msg.position[2]) + 90)
        self.roll_pulse2 = int(self.DegToPulse(tempDegree))
        tempDegree = trimLimits(radiansToDegrees(cmd_msg.position[3]) + 90)
        self.roll_pulse3 = int(self.DegToPulse(tempDegree))
        tempDegree = trimLimits(radiansToDegrees(cmd_msg.position[4]) + 90)
        self.pitch_pulse = int(self.DegToPulse(tempDegree))
        #tempDegree = trimLimits(radiansToDegrees(cmd_msg.position[5]))
        #self.gripper_pulse = int(self.DegToPulse(tempDegree))

        self.motor0.run(self.yaw_pulse)       
        self.motor1.run(self.roll_pulse1)     
        self.motor2.run(self.roll_pulse2)     
        self.motor3.run(self.roll_pulse3)     
        self.motor4.run(self.pitch_pulse)     
        self.motor5.run(self.gripper_pulse)   
        
        self.timediff = time() - self.prev_time
        self.prev_time = time()

        print( str(self.yaw_pulse) + ':' + str(self.roll_pulse1) + ':' + str(self.roll_pulse2) 
        + ':' + str(self.roll_pulse3) + ':' + str(self.pitch_pulse) + ':' + str(self.gripper_pulse) + ':' + str(self.timediff))

if __name__ == "__main__":

    rospy.init_node("chase_moveit_control")
    myArm = RobotArm("donkey_arm")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    myArm.__del__()
