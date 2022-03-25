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
import math, time
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from rospy.topics import Message

import myconfig as mc
import myutil as mu

class ChaseObject:
    def __init__(self):

        self.blob_x = 0.0
        self.blob_y = 0.0
        self.prev_steer_action = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._time_detected = 0.0

        self.sub_center = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.update_object)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/dkcar/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message = Twist()

        self._time_steer = 0
        self._steer_sign_prev = 0

    @property
    def is_detected(self):
        return time.time() - self._time_detected < 1.0

    def update_object(self, message):
        for box in message.bounding_boxes:
            #
            #yolov4-tiny, 416x416
            if box.Class == mc.DETECT_CLASS:
                self.blob_x = float((box.xmax + box.xmin)/mc.PICTURE_SIZE_X/2.0) - 0.5
                self.blob_y = float((box.ymax + box.ymin)/mc.PICTURE_SIZE_Y/2.0) - 0.5
                self._time_detected = time.time()
                rospy.loginfo("Yolo X,Y(-1 ~ 1): %.2f  %.2f "%(self.blob_x, self.blob_y))
                #rospy.loginfo(
                #    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {} Class: {}".format
                #    (box.xmin, box.xmax, box.ymin, box.ymax, box.Class) )
            
    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        steer_action = 0.0
        object_detect = 0.0
        final_steer_action = 0.0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            steer_action = mc.K_LAT_DIST_TO_STEER * self.blob_x
            
            #PI controller      
            #AVG(prev_steer_action)*Ki + steer_action*Kp
            final_steer_action = sum(self.prev_steer_action)/len(self.prev_steer_action)*mc.Ki + steer_action*mc.Kp
            final_steer_action = mu.clamp(final_steer_action, -1.0, 1.0)
            #shift left once, add last item
            self.prev_steer_action = self.prev_steer_action[1:] + self.prev_steer_action[:1]
            self.prev_steer_action[4] = steer_action

            if ((steer_action > mc.IN_RANGE_MIN) and (steer_action < mc.IN_RANGE_MAX)) :
                self.prev_steer_action = [0.0, 0.0, 0.0, 0.0, 0.0]
                final_steer_action = 0
                
            object_detect = 1.0
            #rospy.loginfo("isDetected, Steering = %2.2f, Current Steer = %2.2f" % (final_steer_action, steer_action))           

        return (final_steer_action, object_detect)

    def run(self):

        # --- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # -- Get the control action
            steer_action, object_detect = self.get_control_action()
            #rospy.loginfo("RUN, Steering = %3.1f Detected = %3.1f" % (steer_action, object_detect))

            # -- update the message
            self._message.linear.x = object_detect
            self._message.angular.z = steer_action

            # -- publish it
            if self.is_detected:
                rospy.loginfo("Steering = %.2f, object_detect = %.2f", steer_action, object_detect)
                self.pub_twist.publish(self._message)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("chase_object_yolo")
    chase_ball = ChaseObject()
    chase_ball.run()
