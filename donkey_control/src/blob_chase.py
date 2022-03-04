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
import time

STEER_CENTER=380
STEER_LIMIT = 110

class PCA9685:
    """
    PWM motor controller using PCA9685 boards.
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

class ServoConvert:
    def __init__(self, id=1, center_value=STEER_CENTER, range=STEER_LIMIT*2, direction=1):
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


def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value


class RobotArm:
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        # --- Initialize the node
        rospy.init_node("blob_chase_node")

        self._steering = PCA9685(channel=0, address=0x40, busnum=1)
        self._tilt     = PCA9685(channel=1, address=0x40, busnum=1)
        #self.motor2 = PCA9685(channel=2, address=0x40, busnum=1)
        #self.motor3 = PCA9685(channel=3, address=0x40, busnum=1)
        #self.motor4 = PCA9685(channel=14, address=0x40, busnum=1)
        #self.motor5 = PCA9685(channel=15, address=0x40, busnum=1)
        rospy.loginfo("PCA9685 Awaked!!")

        self.actuators = {}
        self.actuators["steering"] = ServoConvert(
            id=1, center_value=STEER_CENTER, range=STEER_LIMIT*2, direction=1
        )
        self.actuators["tilt"] = ServoConvert(
            id=2, center_value=STEER_CENTER, range=STEER_LIMIT*2, direction=1
        )  # -- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        # --- Create a debug publisher for resulting cmd_vel
        self.ros_pub_debug_command = rospy.Publisher(
            "/dkcar/debug/cmd_vel", Twist, queue_size=1
        )
        rospy.loginfo("> Publisher corrrectly initialized")

        # --- Create the Subscriber to obstacle_avoidance commands
        self.ros_sub_twist = rospy.Subscriber(
            "/dkcar/control/cmd_vel", Twist, self.update_message_from_chase
        )
        rospy.loginfo("> Subscriber corrrectly initialized")

        self.tilt_chase = 0.0
        self.steer_chase = 0.0

        self._debud_command_msg = Twist()

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._last_time_chase_rcv = time.time()
        self._timeout_ctrl = 100
        self._timeout_blob = 1

        rospy.loginfo("Initialization complete")

    def update_message_from_chase(self, message):
        self._last_time_chase_rcv = time.time()
        self.tilt_chase = message.linear.x
        self.steer_chase = message.angular.z
        print(self.tilt_chase, self.steer_chase)

    def compose_command_velocity(self):
        self.tilt = saturate(self.tilt_chase, -1, 1)
        self.steer = saturate(self.steer_chase, -1, 1)
        #self._debud_command_msg.linear.x = self.tilt
        #self._debud_command_msg.angular.z = self.steer
        #self.ros_pub_debug_command.publish(self._debud_command_msg)
        self.set_actuators_from_cmdvel(self.tilt, self.steer)

    def set_actuators_from_cmdvel(self, tilt, steering):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Convert vel into servo values
        self.actuators["tilt"].get_value_out(tilt)
        self.actuators["steering"].get_value_out(steering)
        # rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(tilt, steering))

        self.set_pwm_pulse(self.actuators["tilt"].value_out, self.actuators["steering"].value_out)

        print( "tilt: " + str(self.actuators["tilt"].value_out) +  ", steering: " + str(self.actuators["steering"].value_out))


    def set_pwm_pulse(self, speed_pulse, steering_pulse):
        self._tilt.run(speed_pulse)
        self._steering.run(steering_pulse)

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.tilt_cmd = 0.0
        self.steer_cmd = 0.0

    def reset_avoid(self):
        self.tilt_chase = 0.0
        self.steer_avoid = 0.0

    @property
    def is_controller_connected(self):
        # print time.time() - self._last_time_cmd_rcv
        return time.time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time.time() - self._last_time_chase_rcv < self._timeout_blob

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


if __name__ == "__main__":
    myArm = RobotArm()
    myArm.run()
