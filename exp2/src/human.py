#!/usr/bin/env python

## @package human
# Emulates a human agent which throws or hides a ball.
# The user randomly chooses what to do with the ball;
# when they have chosen, is sends a goal action to go_to_point_ball.py

import rospy
import time
import random
import math
import exp2.msg
import actionlib
import actionlib.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

## Acquire parameters from the launch file

map_x_max = rospy.get_param('map/x_max')
map_y_max = rospy.get_param('map/y_max')

map_x_min = rospy.get_param('map/x_min')
map_y_min = rospy.get_param('map/y_min')

sim_scale = rospy.get_param('sim_scale')


#Randomize the human behaviour
def human():
    rospy.init_node('human_node', anonymous=True)
    ball_action_client = actionlib.SimpleActionClient('/ball/reaching_goal_topic', exp2.msg.PlanningAction)
    rate = rospy.Rate(200)
    ball_choice = PoseStamped()
    while not rospy.is_shutdown():
        time.sleep(random.randint(30, 70) / sim_scale)
        if rospy.get_param('state') == 'normal':
            human_moves_timer = math.floor(random.randint(20, 40) / sim_scale)
            last_choice = 0
            while (rospy.get_param('state') != 'sleep' and human_moves_timer != 0):
                ball_choice.pose.position.x = random.randint(map_x_min, map_x_max)
                ball_choice.pose.position.y = random.randint(map_y_min, map_y_max)
                hide_variable = random.randint(0, 1)
                if hide_variable == 0:
                    ball_choice.pose.position.z = 0.5
                    rospy.loginfo('*The human moves the ball to: %i %i*', \
                      ball_choice.pose.position.x, ball_choice.pose.position.y)             
                    ball_goal = exp2.msg.PlanningGoal(target_pose = ball_choice)
                    ball_action_client.send_goal(ball_goal)
                    ball_action_client.wait_for_result()
                    rospy.loginfo('Ball moved to: %i %i*', \
                      ball_choice.pose.position.x, ball_choice.pose.position.y)
                    last_choice = 0
                else:
                    if last_choice == 1:
                        rospy.loginfo('*The human keeps the ball hidden*') 
                        time.sleep(random.randint(6, 7) / sim_scale)
                    else:
                        ball_choice.pose.position.z = -0.5
                        rospy.loginfo('*The human hides the ball...*')                  
                        ball_goal = exp2.msg.PlanningGoal(target_pose = ball_choice)
                        ball_action_client.send_goal(ball_goal)
                        ball_action_client.wait_for_result()
                        rospy.loginfo('Ball hidden!')
                        last_choice = 1
                human_moves_timer = human_moves_timer - 1

                time.sleep(random.randint(10, 16) / sim_scale)
            rate.sleep()
            ball_choice.pose.position.x = random.randint(map_x_min, map_x_max)
            ball_choice.pose.position.y = random.randint(map_y_min, map_y_max)
            ball_choice.pose.position.z = -0.5
            rospy.loginfo('*The human hides the ball and stops for a while...*')              
            ball_goal = exp2.msg.PlanningGoal(target_pose = ball_choice)
            ball_action_client.send_goal(ball_goal)
            ball_action_client.wait_for_result()
            rospy.loginfo('Ball hidden for a while!')
    rate.sleep()

if __name__ == '__main__':
    try:
        human()
    except rospy.ROSInterruptException:
        pass
