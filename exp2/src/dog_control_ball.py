#!/usr/bin/env python

#This let the robot follOW the ball in the PLAY state


import sys
import time
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2

import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

from std_msgs.msg import Int64, Float64



find_counter = 0
head_moved = 0

sim_scale = rospy.get_param('sim_scale')

# To control the robot in the PLAY state
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('dog_control_ball_node', anonymous=True)

        
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        self.ball_pub = rospy.Publisher("ball_control_topic", Int64, queue_size=1) 

        
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.head_pub = rospy.Publisher("joint_position_controller/command", Float64, queue_size=1)

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        global find_counter, head_moved

        if (rospy.get_param('state') == 'normal' or rospy.get_param('state') == 'play'):
           
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 

            head_angle = Float64()

            greenLower = (50, 50, 20)
            greenUpper = (70, 255, 255)

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            
            if len(cnts) > 0:
                if not rospy.get_param('state') == 'sleep':
                    rospy.set_param('ball_detected', 1)
                find_counter = 0
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                self.ball_pub.publish(1)

                if radius > 105:
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
                    if rospy.get_param('state') == 'play':
                        vel = Twist()
                        vel.angular.z = -0.002 * (center[0] - 400)
                        vel.linear.x = -0.01 * (radius - 100)
                        self.vel_pub.publish(vel)
                elif (radius < 95 and rospy.get_param('state') == 'play'):
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if vel.linear.x < 0.4:
                        vel.linear.x = 0.4
                    elif vel.linear.x > 1.0:
                        vel.linear.x = 1.0
                    if vel.angular.z < 0.1:
                        vel.angular.z = 0.1
                    elif vel.angular.z > 0.3:
                        vel.angular.z = 0.3
                    self.vel_pub.publish(vel)
                elif (rospy.get_param('state') == 'play' and radius <= 105 and radius >= 95):
                    if head_moved == 0:
                        rospy.loginfo('*The dog is turning the head...*')
                        head_angle = 0
                        vel = Twist()
                        vel.linear.x = 0
                        vel.angular.z = 0
                        self.vel_pub.publish(vel)
                        self.head_pub.publish(head_angle)
                        while head_angle < (0.7854):
                            head_angle = head_angle + 0.1
                            self.head_pub.publish(head_angle)
                            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                            cv2.imshow('window', image_np)
                            time.sleep(1 / sim_scale)
                        time.sleep(1 / sim_scale)
                        while head_angle > (-0.7854):
                            head_angle = head_angle - 0.1
                            self.head_pub.publish(head_angle)
                            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                            cv2.imshow('window', image_np)
                            time.sleep(1 / sim_scale)
                        time.sleep(1 / sim_scale)
                        while head_angle < 0:
                            head_angle = head_angle + 0.1
                            self.head_pub.publish(head_angle)
                            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                            cv2.imshow('window', image_np)
                            time.sleep(1 / sim_scale)
                        head_angle = 0
                        self.head_pub.publish(head_angle)   
                        head_moved = 1              

            elif (rospy.get_param('state') == 'play' and find_counter < 100):
                head_moved = 0
                vel = Twist()
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                find_counter = find_counter + 1
            elif (rospy.get_param('state') == 'play' and find_counter >= 100):
                head_moved = 0
                vel = Twist()
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                self.ball_pub.publish(2)
                rospy.set_param('ball_detected', 0)

            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)


def main(args):
    '''Initializes and cleanups ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
