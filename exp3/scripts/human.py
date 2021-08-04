#!/usr/bin/env python


# This code is responsible to make the human randomly play with the dog and to sent it to different rooms. 
# All the communications sent to the robot are carried out via action a message publisher (over 'play_topic'), and all the orders are handled by dog_fsm.py

import rospy
import time
import random
from std_msgs.msg import String

## Acquire the list of available rooms from launch file
room_list = rospy.get_param('room_list')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')


# Function to send the Robot to randomly chosen rooms by Human.
def human():
    rospy.init_node('human_node', anonymous = True)
    rate = rospy.Rate(200)

    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    time.sleep(random.randint(80, 120) / sim_scale)
    while not rospy.is_shutdown():
        while (rospy.get_param('state') == 'sleep' or rospy.get_param('state') == 'find'):
            rate.sleep()
        if rospy.get_param('state') == 'normal':
            voice_pub.publish('play')
            rospy.loginfo('Human: I want to play')
            while (rospy.get_param('state') != 'play'):
                rate.sleep()

        while (rospy.get_param('state') == 'play' or rospy.get_param('state') == 'find'):
            while (rospy.get_param('play_task_status') ==  0):
                rate.sleep()
            room_choice = random.randint(0, 5)
            rospy.loginfo('Human: Go to the %s', room_list[room_choice])
            voice_pub.publish(room_list[room_choice])
            # when 'play_task_status' equals 2, the robot has completed its job
            while (rospy.get_param('play_task_status') !=  2):
                rate.sleep()
            rate.sleep()
        rospy.loginfo('Human: The robot has stopped playing')
        time.sleep(random.randint(250, 300) / sim_scale)
        rate.sleep()

if __name__ == '__main__':
    try:
        human()
    except rospy.ROSInterruptException:
        pass
