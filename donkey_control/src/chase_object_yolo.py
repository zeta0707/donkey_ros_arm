#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to 
    /darknet_ros/bounding_boxes
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import math
import rospy
from time import time, sleep
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from rospy.topics import Message

import myconfig as mc
from myutil import clamp

class ChaseObject:
    def __init__(self):

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0

        self.sub_center = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.update_object)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/dkcar/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message = Twist()
        self._time_steer = 0

        self.yolo_target1 = rospy.get_param("/jessiarm_target/DETECT_CLASS1")
        self.yolo_target2 = rospy.get_param("/jessiarm_target/DETECT_CLASS2")
        self.yolo_target = 0

    @property
    def is_detected(self):
        return time() - self._time_detected < 0.2

    def update_object(self, message):
        for box in message.bounding_boxes:
            #yolov4-tiny, 416x416
            if (box.Class == self.yolo_target1) or (box.Class == self.yolo_target2):
                self.blob_x = float((box.xmax + box.xmin)/mc.PICTURE_SIZE_X/2.0) - 0.5 - mc.CALIBPICTURE
                #self.blob_y = float((box.ymax + box.ymin)/mc.PICTURE_SIZE_Y/2.0) - 0.5
                self._time_detected = time()
                rospy.loginfo("Yolo X,Y(-1 ~ 1): %.2f  %.2f "%(self.blob_x, self.blob_y))
                if box.Class == self.yolo_target1:
                    self.yolo_target = 1
                else:
                    self.yolo_target = 2
                #rospy.loginfo(
                #    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {} Class: {}".format
                #    (box.xmin, box.xmax, box.ymin, box.ymax, box.Class) )
            else:
                self.yolo_target = 0
                rospy.loginfo("Yolo different object")
            
    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        steer_action = 0.0
        object_detect = 0.0
        final_steer_action = 0.0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            steer_action = mc.DIR_TO_STEER * self.blob_x            
            final_steer_action =  steer_action*mc.Kp
            final_steer_action = clamp(final_steer_action, -1.0, 1.0)

            if ((steer_action > mc.IN_RANGE_MIN) and (steer_action < mc.IN_RANGE_MAX)) :
                final_steer_action = 0
                
            object_detect = self.yolo_target
            #rospy.loginfo("isDetected, Steering = %2.2f, Current Steer = %2.2f" % (final_steer_action, steer_action))           

        return (object_detect, final_steer_action)

    def run(self):

        # --- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # -- Get the control action
            object_detect, steer_action  = self.get_control_action()
            #rospy.loginfo("RUN, Steering = %3.1f Detected = %3.1f" % (steer_action, object_detect))

            # -- update the message
            self._message.linear.x = object_detect
            self._message.angular.z = steer_action

            # -- publish it
            #rospy.loginfo("Steering = %.2f, object_detect = %.2f", steer_action, object_detect)
            self.pub_twist.publish(self._message)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("chase_yolo")
    chase_ball = ChaseObject()
    chase_ball.run()
