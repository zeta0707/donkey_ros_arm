#!/usr/bin/env python

"""
referenced from those projects

DkLowLevelCtrl, ServoConvert part from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

PCA9685 part from donkeycar
url: https://github.com/autorope/donkeycar/blob/99c853b1737f12019ae598d3c7f00699d2166472/donkeycar/parts/actuator.py#L12

Listens to /dkcar/control/cmd_vel for corrective actions to the /cmd_vel coming from keyboard or joystick

"""
import rospy
from geometry_msgs.msg import Twist
from time import time, sleep

import myconfig as mc
from myutil import clamp, clampRem, PCA9685

class ServoConvert:
    def __init__(self, id=1, center_value=mc.MOTOR0_HOME, range=mc.MOTOR0_RANGE, direction=1):
        self.value = 0.0
        self.value_out = center_value
        self._center = center_value
        self._range = range
        self._half_range = 0.5 * range # 45
        self._dir = direction # 1 or -1 
        self.id = id

        # --- Convert its range in [-1, 1]
        self._sf = 1.0 / self._half_range # 1 / 45

    def get_value_out(self, value_in):
        # --- twist type value is in  [-1, 1]
        self.value = value_in
        self.value_out = int(self._dir * value_in * self._half_range + self._center)
        return self.value_out

class RobotArm:
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")
        # --- Initialize the node
        rospy.init_node("blob_control")
        
        self.motor0 = PCA9685(channel=0, address=0x40, busnum=1)
        self.motor1 = PCA9685(channel=1, address=0x40, busnum=1)
        self.motor2 = PCA9685(channel=2, address=0x40, busnum=1)
        self.motor3 = PCA9685(channel=3, address=0x40, busnum=1)
        self.motor4 = PCA9685(channel=14, address=0x40, busnum=1)
        self.motor5 = PCA9685(channel=15, address=0x40, busnum=1)

        rospy.loginfo("PCA9685 Awaked!!")

        self.armStatus = "Homing"

        # start from off position
        self.goPosOff()

        # move to home positon from off position
        self.motor1.runTarget(mc.MOTOR1_OFF, mc.MOTOR1_HOME)
        self.motor2.runTarget(mc.MOTOR2_OFF, mc.MOTOR2_HOME)
        self.motor3.runTarget(mc.MOTOR3_OFF, mc.MOTOR3_HOME)    
        self.motor4.runTarget(mc.MOTOR4_OFF, mc.MOTOR4_HOME)  
        self.motor5.runTarget(mc.GRIPPER_OFF, mc.GRIPPER_HOME) 
        self.motor0.runTarget(mc.MOTOR0_OFF, mc.MOTOR0_HOME)        

        self.actuators = {}
        self.actuators["steering"] = ServoConvert(
            id=1, center_value=mc.MOTOR0_HOME, range=mc.MOTOR0_RANGE, direction=1
        )
        rospy.loginfo("> steering corrrectly initialized")

        # --- Create the Subscriber to object following
        self.ros_sub_twist = rospy.Subscriber(
            "/dkcar/control/cmd_vel", Twist, self.update_message_from_chase
        )
        rospy.loginfo("> Subscriber corrrectly initialized")

        self.object_detect = 0.0
        # free run command
        self.steer_cmd = 0.0
        # steer from object cord
        self.steer_chase = 0.0
        self.steer_cmd_dir = 1.00

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time()
        self._last_time_chase_rcv = time()
        self._timeout_ctrl = 100
        self._timeout_blob = 1
        self.armStatus = "Scanning"

        rospy.loginfo("Initialization complete")

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

    def pickup(self, xPos):
        self.armStatus = "PickingUp"
        self.motor3.run(mc.MOTOR3_PICKUP)
        sleep(0.5)
        self.motor5.run(mc.GRIPPER_CLOSE) 
        sleep(0.5)
        self.motor2.run(mc.MOTOR2_PICKUP)
        sleep(0.5)
        self.motor0.runTarget(xPos, mc.YAW_MIN) 
        sleep(0.5)
        self.motor1.runTarget(mc.MOTOR1_OFF, mc.MOTOR1_HOME)
        sleep(0.5)
        self.motor2.runTarget(mc.MOTOR2_PICKUP, mc.MOTOR2_HOME)
        sleep(0.5)        
        self.motor5.runTarget(mc.GRIPPER_CLOSE, mc.GRIPPER_OPEN)   
        sleep(0.5)
        self.motor0.runTarget(mc.YAW_MIN, mc.MOTOR0_HOME)
        sleep(1) 
        self.motor3.runTarget(mc.MOTOR3_PICKUP, mc.MOTOR3_HOME)    
        sleep(0.5)

    def update_message_from_chase(self, message):
        self._last_time_chase_rcv = time()
        self.object_detect = message.linear.x
        self.steer_chase = message.angular.z
        #print(self.object_detect, self.steer_chase)

    def compose_command_velocity(self):
        #steer_cmd: free run, steer_chase: from object detection 
        if (self.object_detect == 1.00):
            self.steer=self.steer_chase  
        # if object is not detected, free run by adding steer_cmd
        else:                                  
            self.steer_cmd += mc.SCAN_SPEED*self.steer_cmd_dir
            self.steer_cmd = clamp(self.steer_cmd, -1.00, 1.00) 
            self.steer=self.steer_cmd
            if (self.steer_cmd == 1.00):
                self.steer_cmd_dir = -1.00
            elif (self.steer_cmd == -1.00):
                self.steer_cmd_dir = 1.00     

        #rospy.loginfo("Got a command = %2.2f  dir = %2.2f"%(self.steer_cmd, self.steer_cmd_dir)) 
        self.set_actuators_from_cmdvel(self.object_detect, self.steer)

    def set_actuators_from_cmdvel(self, object_detect, steering):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Convert vel into servo values
        self.actuators["steering"].get_value_out(steering)
        rospy.loginfo("Got a command det = %2.2f  steer = %2.2f"%(object_detect, steering))    
        #check object is in range, then pick it up
        if self.armStatus == "Scanning":
            if ((steering > mc.IN_RANGE_MIN) and (steering < mc.IN_RANGE_MAX) and (object_detect == 1.0)):
                print("Object in front,pick it up")
                self.pickup(self.actuators["steering"].get_value_out(steering))
                self.armStatus = "Scanning"
            #else free run or move to target
            else :
                self.set_pwm_pulse(self.actuators["steering"].value_out)
                
    def set_pwm_pulse(self, steering_pulse):
        self.motor0.run(steering_pulse)

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.object_detect = 0.0
        #self.steer_cmd = 0.0

    def reset_avoid(self):
        self.object_detect = 0.0
        #self.steer_cmd = 0.0

    @property
    def is_controller_connected(self):
        # print time.time() - self._last_time_cmd_rcv
        return time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time() - self._last_time_chase_rcv < self._timeout_blob

    def run(self):
        # --- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.compose_command_velocity()

            if not self.is_controller_connected:
                self.set_actuators_idle()

            if not self.is_chase_connected:
                self.reset_avoid()
            rate.sleep()
        self.__del__()

if __name__ == "__main__":
    myArm = RobotArm()
    myArm.run()
