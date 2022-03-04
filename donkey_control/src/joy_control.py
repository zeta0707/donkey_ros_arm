#!/usr/bin/env python

"""
Node for control PCA9685 using AckermannDriveStamped msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

import time
import rospy
from threading import Thread
from ackermann_msgs.msg import AckermannDriveStamped

STEER_CENTER=380
STEER_LIMIT=110

yaw_pulse = STEER_CENTER
roll_pulse = STEER_CENTER
roll_pulse1 = STEER_CENTER
roll_pulse2 = STEER_CENTER
pitch_pulse = STEER_CENTER
gripper_pulse = STEER_CENTER - STEER_LIMIT

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
        pulse_diff = pulse - self.prev_pulse
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
            "/donkey_teleop",
            AckermannDriveStamped,
            self.joy_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.motor0.run(STEER_CENTER)
        self.motor1.run(STEER_CENTER)
        self.motor2.run(STEER_CENTER)
        self.motor3.run(STEER_CENTER+20)             #right angle
        self.motor4.run(STEER_CENTER+20)             #right angle
        self.motor5.run(STEER_CENTER-STEER_LIMIT)    #gripper open
        rospy.loginfo("Teleop Subscriber Awaked!! Waiting for joystick...")

    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)

    def joy_callback(self, msg):
        global yaw_pulse
        global roll_pulse  
        global roll_pulse1
        global roll_pulse2
        global pitch_pulse
        global gripper_pulse

        yaw_pulse += int((msg.drive.steering_angle)/1024)
        roll_pulse += int((msg.drive.jerk)/1024)
        pitch_pulse += int((msg.drive.speed)/1024)
        gripper_pulse += int((msg.drive.acceleration)/1024)

        if pitch_pulse > (STEER_CENTER + STEER_LIMIT) :
           pitch_pulse = STEER_CENTER + STEER_LIMIT
        elif pitch_pulse < (STEER_CENTER - STEER_LIMIT) :
           pitch_pulse = STEER_CENTER - STEER_LIMIT

        if yaw_pulse > (STEER_CENTER + STEER_LIMIT) :
           yaw_pulse = STEER_CENTER + STEER_LIMIT
        elif yaw_pulse < (STEER_CENTER - STEER_LIMIT) :
           yaw_pulse = STEER_CENTER - STEER_LIMIT

        if gripper_pulse > STEER_CENTER :
           gripper_pulse = STEER_CENTER
        elif gripper_pulse < (STEER_CENTER - STEER_LIMIT) :
           gripper_pulse = STEER_CENTER - STEER_LIMIT

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
            + "motor4_pulse : "
            + str(pitch_pulse)  
            + " / "
            + "motor5_pulse : "
            + str(gripper_pulse)        
        )

        self.motor0.run(yaw_pulse)      #control by joystick
        self.motor1.run(roll_pulse1)    #control by joystick
        self.motor2.run(roll_pulse2)    #control by joystick
        self.motor4.run(pitch_pulse)    #control by joystick
        self.motor5.run(gripper_pulse)    #control by joystick

if __name__ == "__main__":

    rospy.init_node("donkey_control")
    myArm = RobotArm("donkey_ros")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
