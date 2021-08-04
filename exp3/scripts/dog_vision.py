#!/usr/bin/env python

## @package dog_vision
# Implements a vision module for the robotic dog that uses OpenCV in order to constantly
# scan the surroundings, looking for specific colored balls.
# This node is able to take control, when needed, of the robot movements, allowing it
# to reach a room when a corresponding new ball is discovered. It also stores
# the positions of the balls discovered, thus learning the displacement of the rooms
# inside the house as time passes

# Python library
import sys

# numpy
import numpy as np

import imutils

# OpenCV
import cv2

# Ros library
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import actionlib
from exp3.srv import Explore
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction

## set lower bound of the BGR color coding for blue ball recognition
blueLower = (100, 50, 50)
## set upper bound of the BGR color coding for blue ball recognition
blueUpper = (130, 255, 255)
## set lower bound of the BGR color coding for red ball recognition
redLower = (0, 50, 50)
## set upper bound of the BGR color coding for red ball recognition
redUpper = (5, 255, 255)
## set lower bound of the BGR color coding for green ball recognition
greenLower = (50, 50, 50)
## set upper bound of the BGR color coding for green ball recognition
greenUpper = (70, 255, 255)
## set lower bound of the BGR color coding for yellow ball recognition
yellowLower = (25, 50, 50)
## set upper bound of the BGR color coding for yellow ball recognition
yellowUpper = (35, 255, 255)
## set lower bound of the BGR color coding for magenta ball recognition
magentaLower = (125, 50, 50)
## set upper bound of the BGR color coding for magenta ball recognition
magentaUpper = (150, 255, 255)
## set lower bound of the BGR color coding for black ball recognition
blackLower = (0, 0, 0)
## set upper bound of the BGR color coding for black ball recognition
blackUpper = (5, 50, 50)

## defines the current knowledge state of the coordinates corresponding to the blue ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
blue_solved = 0
## defines the current knowledge state of the coordinates corresponding to the red ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
red_solved = 0
## defines the current knowledge state of the coordinates corresponding to the green ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
green_solved = 0
## defines the current knowledge state of the coordinates corresponding to the yellow ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
yellow_solved = 0
## defines the current knowledge state of the coordinates corresponding to the magenta ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
magenta_solved = 0
## defines the current knowledge state of the coordinates corresponding to the black ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
black_solved = 0

## Class used to visualize what the the robotic dog see during motion. It uses
# OpenCV in order to acquire images of the house, and takes control of the
# robot movements as soon as a new ball is spotted, with the aim of getting near it
# and storing the location for later reference
class image_feature:

    ## Class initialization: subscribe to topics and declare publishers
    def __init__(self):
        rospy.init_node('dog_vision_node', anonymous = True)

        ## topic over which camera images get published
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        ## topic used to publish robot velocity commands
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        ## subscribed topic, used to check if balls are present nearby
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    ## Function used to compute the amount of image contours comprised inside
    # a specific color's bounds
    def computeCount(self, hsv, colorLower, colorUpper):
            colorMask = cv2.inRange(hsv, colorLower, colorUpper)
            colorMask = cv2.erode(colorMask, None, iterations=2)
            colorMask = cv2.dilate(colorMask, None, iterations=2)
            colorCnts = cv2.findContours(colorMask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            colorCnts = imutils.grab_contours(colorCnts)
            return colorCnts

    ## Callback of the subscribed topic, used to constantly show on screen what
    # the robotic dog is seeing. This callback also performs image conversion
    # and feature detection. It has a different algorithm depending on the robotic
    # dog current state. When the robot is in the Sleep or Play state, this callback
    # just provides visual feedback on screen about what the robot is currently seeing.
    # With this choice, the robot's Play state actually prioritizes the current human
    # request over any possible new ball detection, i.e. they are ignored.
    # When the robot is in Normal state, this callback does nothing if no balls are
    # detected. If this is not the case, the robot checks if that ball is present
    # in its database and, if so, it ignores it. If the ball preceived is a new discovery,
    # the callback cancels all current move_base goals through an action client, and
    # takes control of the dog motion: it leads the robot close to the ball,
    # and then stores its coordinates. While this process is being carried out,
    # the code draws both a colored dot centered on the ball and a circle outlining the
    # latter; the callback also locks the triggering of new ball discovery processes,
    # and frees them only once it is completed. The callback behavior for the Find state
    # is similar to the one of the Normal state, but this time the goal cancelling
    # is sent to the explore_lite algorithm via a service client. The code also
    # checks if the ball found and reached was actually the wanted one:
    # if so, the finding process is over, otherwise the callback restarts the
    # explore_lite algorithm, with another service client, in order to find
    # another new ball. The two service clients used to communicate with the Explore
    # class (which hosts the exploration algorithm) rely on as many service servers,
    # that I added as methods of the original code
    def callback(self, ros_data):
        global blue_solved, red_solved, green_solved, \
                yellow_solved, magenta_solved, black_solved

        ## Direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:



        if (rospy.get_param('state') == 'normal'):

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            blueCnts = image_feature.computeCount(self, hsv, blueLower, blueUpper)
            redCnts = image_feature.computeCount(self, hsv, redLower, redUpper)
            greenCnts = image_feature.computeCount(self, hsv, greenLower, greenUpper)
            yellowCnts = image_feature.computeCount(self, hsv, yellowLower, yellowUpper)
            magentaCnts = image_feature.computeCount(self, hsv, magentaLower, magentaUpper)
            blackCnts = image_feature.computeCount(self, hsv, blackLower, blackUpper)

            center = None
            # only proceed if at least one contour was found

            if(len(blueCnts) > 0 and blue_solved != 2 \
                     and red_solved != 1 and green_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                blue_solved = 1
                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the entrance position (blue ball)')
                    blue_solved = 2
                    rospy.set_param('new_ball_detected', 0)

            elif(len(redCnts) > 0 and red_solved != 2 \
                     and blue_solved != 1 and green_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                red_solved = 1
                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the closet position (red ball)')
                    red_solved = 2
                    rospy.set_param('new_ball_detected', 0)

            elif(len(greenCnts) > 0 and green_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                green_solved = 1
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the livingroom position (green ball)')
                    green_solved = 2
                    rospy.set_param('new_ball_detected', 0)

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                yellow_solved = 1
                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a magenta circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the kitchen position (yellow ball)')
                    yellow_solved = 2
                    rospy.set_param('new_ball_detected', 0)

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1
                     and yellow_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                magenta_solved = 1
                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bathroom position (magenta ball)')
                    magenta_solved = 2
                    rospy.set_param('new_ball_detected', 0)

            elif(len(blackCnts) > 0 and black_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1
                     and yellow_solved != 1 and magenta_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                black_solved = 1
                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bedroom position (black ball)')
                    black_solved = 2
                    rospy.set_param('new_ball_detected', 0)



        elif(rospy.get_param('state') == 'find'):

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            blueCnts = image_feature.computeCount(self, hsv, blueLower, blueUpper)
            redCnts = image_feature.computeCount(self, hsv, redLower, redUpper)
            greenCnts = image_feature.computeCount(self, hsv, greenLower, greenUpper)
            yellowCnts = image_feature.computeCount(self, hsv, yellowLower, yellowUpper)
            magentaCnts = image_feature.computeCount(self, hsv, magentaLower, magentaUpper)
            blackCnts = image_feature.computeCount(self, hsv, blackLower, blackUpper)

            center = None
            # only proceed if at least one contour was found

            if(len(blueCnts) > 0 and blue_solved != 2 \
                     and red_solved != 1 and green_solved != 1 and yellow_solved != 1
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                blue_solved = 1
                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the entrance position (blue ball)')
                    blue_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 0:
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)

            elif(len(redCnts) > 0 and red_solved != 2 \
                     and blue_solved != 1 and green_solved != 1 and yellow_solved != 1
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                red_solved = 1
                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the closet position (red ball)')
                    red_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 1:
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)

            elif(len(greenCnts) > 0 and green_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                green_solved = 1
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the livingroom position (green ball)')
                    green_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 2:
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                yellow_solved = 1
                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a magenta circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the kitchen position (yellow ball)')
                    yellow_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 3:
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                     and yellow_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                magenta_solved = 1
                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bathroom position (magenta ball)')
                    magenta_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 4:
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)

            elif(len(blackCnts) > 0 and black_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                     and yellow_solved != 1 and magenta_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                black_solved = 1
                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bedroom position (black ball)')
                    black_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 5:
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)



        cv2.imshow('window', image_np)
        cv2.waitKey(2)

## Invokes the image_feature class and spins until interrupted by a keyboard command
def main(args):
    #Initializes and cleanups ros node
    #ic = image_feature()
    image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
