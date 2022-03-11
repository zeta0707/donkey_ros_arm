#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

K_LAT_DIST_TO_STEER = -1.0

import myconfig as mc
import myutil as mu

class ChaseBall:
    def __init__(self):

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0

        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/dkcar/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message = Twist()

        self._time_steer = 0
        self._steer_sign_prev = 0

    @property
    def is_detected(self):
        return time.time() - self._time_detected < 1.0

    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self._time_detected = time.time()
        # rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        steer_action = 0.0
        tilt_action = 0.0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            steer_action = K_LAT_DIST_TO_STEER * self.blob_x
            tilt_action = 1.0
            rospy.loginfo("Steering command %.2f" % steer_action)
            #if object is detected, go forward with defined power
        return (steer_action, tilt_action)

    def run(self):

        # --- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            # -- Get the control action
            steer_action, tilt_action = self.get_control_action()
            
            # -- update the message
            self._message.linear.x = tilt_action
            self._message.angular.z = steer_action

            # -- publish it, only blob detected
            if self.is_detected:
                rospy.loginfo("Steering = %.2f, tilt = %.2f", steer_action, tilt_action)
                self.pub_twist.publish(self._message)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("chase_ball")
    chase_ball = ChaseBall()
    chase_ball.run()
