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

STEER_CENTER=380
STEER_LIMIT=110

yaw_pulse = STEER_CENTER
roll_pulse = STEER_CENTER
roll_pulse1 = STEER_CENTER
roll_pulse2 = STEER_CENTER

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
           self, channel, address, frequency=60, busnum=None, init_delay=0.1
    ):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

        self.pulse = STEER_CENTER
        self.prev_pulse = STEER_CENTER
        self.running = True

    def set_pwm(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pwm(pulse)
        self.prev_pulse = pulse

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)

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
            "/cmd_vel",
            Twist,
            self.keyboard_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.motor3.run(STEER_CENTER+20)            #right angle
        self.motor4.run(STEER_CENTER+20)            #right angle
        self.motor5.run(STEER_CENTER-STEER_LIMIT)   #gripper open

        rospy.loginfo("Keyboard Subscriber Awaked!! Waiting for keyboard...")

    def keyboard_callback(self, msg):
        global yaw_pulse
        global roll_pulse  
        global roll_pulse1
        global roll_pulse2

        #rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        roll_pulse += msg.linear.x
        yaw_pulse -= msg.angular.z

        if yaw_pulse > (STEER_CENTER + STEER_LIMIT) :
           yaw_pulse = STEER_CENTER + STEER_LIMIT
        elif yaw_pulse < (STEER_CENTER - STEER_LIMIT) :
           yaw_pulse = STEER_CENTER - STEER_LIMIT

        if roll_pulse >  (STEER_CENTER + 2*STEER_LIMIT) :
            roll_pulse =  (STEER_CENTER + 2*STEER_LIMIT)
        elif roll_pulse < (STEER_CENTER - 2*STEER_LIMIT) :
            roll_pulse =  (STEER_CENTER - 2*STEER_LIMIT)

        if roll_pulse >  (STEER_CENTER + STEER_LIMIT) :
            roll_pulse1 = STEER_CENTER + STEER_LIMIT
            roll_pulse2 = STEER_CENTER + roll_pulse - roll_pulse1
        elif roll_pulse <  (STEER_CENTER - STEER_LIMIT) :
            roll_pulse1 = STEER_CENTER - STEER_LIMIT
            roll_pulse2 = STEER_CENTER + roll_pulse - roll_pulse1
        else :
            roll_pulse1 = roll_pulse
            roll_pulse2 =  STEER_CENTER

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
        )

        self.motor0.run(yaw_pulse)      #control by keyboard
        self.motor1.run(roll_pulse1)    #control by keyboard
        self.motor2.run(roll_pulse2)    #control by keyboard


if __name__ == "__main__":

    rospy.init_node("donkey_control")
    myArm = RobotArm("donkey_ros")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()