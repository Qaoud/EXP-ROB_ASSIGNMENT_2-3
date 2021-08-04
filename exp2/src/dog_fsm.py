#!/usr/bin/env python


# This is to describe robot states, which are three main states: SLEEP, NORMAL, AND PLAY

import roslib
import rospy
import smach
import smach_ros
import time
import random
import exp2
from std_msgs.msg import Int64
from exp2.msg import Coordinates


map_x_max = rospy.get_param('map/x_max')
map_y_max = rospy.get_param('map/y_max')

map_x_min = rospy.get_param('map/x_min')
map_y_min = rospy.get_param('map/y_min')


home_x = rospy.get_param('home/x')
home_y = rospy.get_param('home/y')

sim_scale = rospy.get_param('sim_scale')


first_iteration = 1
playtime = 0


#SLEEP STATE FUNCTION

class Sleep(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.sleep_callback)


    def execute(self, userdata):
        global first_iteration
        rospy.set_param('state','sleep')
        pos = Coordinates()
        pos.x = home_x 
        pos.y = home_y 
        pub_sleep = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        if first_iteration == 0:
            rospy.loginfo('dog: I am going to spleep!')
            time.sleep(random.randint(2, 5) / sim_scale)
        else:
            time.sleep(random.randint(2, 5) / sim_scale)
        timer = 1
        while (timer != 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'sleep'):   
            pub_sleep.publish(pos)
            if(rospy.wait_for_message('motion_over_topic', Coordinates)):
                timer = 0
        time.sleep(random.randint(2, 5) / sim_scale) 
        rospy.loginfo('dog: Good morning!')
        return 'wake_up'


    def sleep_callback(self, data):
        global home_x
        global home_y
        global first_iteration
        if (rospy.get_param('state') == 'sleep' and first_iteration == 0):
            rospy.loginfo('dog: home position reached!')
            rospy.set_param('dog/x', home_x)
            rospy.set_param('dog/y', home_y)
        elif(rospy.get_param('state') == 'sleep'):
            rospy.set_param('dog/x', home_x)
            rospy.set_param('dog/y', home_y)
            first_iteration = 0


#NORMAL STATE FUNCTION

class Normal(smach.State):

    def __init__(self):
 
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.normal_callback_motion)
        rospy.Subscriber('ball_control_topic', Int64, self.normal_callback_ball)
    
    def execute(self, userdata):
        global playtime
        sleep_timer = (random.randint(2, 7) / sim_scale)
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'normal')
        pos = Coordinates()
        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        while (sleep_timer != 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'normal' and playtime == 0):
            sleep_timer = sleep_timer - 1
            pos.x = random.randint(map_x_min, map_x_max)
            pos.y = random.randint(map_y_min, map_y_max)
            rospy.loginfo('dog: I am moving to %i %i', pos.x, pos.y)	
            pub.publish(pos)
            if(rospy.wait_for_message('motion_over_topic', Coordinates) or playtime == 1):
                rospy.set_param('dog/x', pos.x)
                rospy.set_param('dog/y', pos.y)
            self.rate.sleep

        if sleep_timer == 0:
            return 'go_sleep'

        elif playtime == 1:
            return 'go_play'


    def normal_callback_motion(self, data):
        if (rospy.get_param('state') == 'normal' and rospy.get_param('ball_detected') == 0):
            rospy.loginfo('dog: %i %i position reached!', data.x, data.y)


    def normal_callback_ball(self, data):
        global playtime
        if (rospy.get_param('state') == 'normal' and rospy.get_param('ball_detected') == 1):
            rospy.loginfo('dog: I have seen the ball! Woof!')
            playtime = 1

#PLAY STATE FUNCTION

class Play(smach.State):

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['game_over'])
        rospy.Subscriber('ball_control_topic', Int64, self.play_callback_ball)

    def execute(self, userdata):
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'play')
        while ((not rospy.is_shutdown()) and playtime == 1 \
            and rospy.get_param('state') == 'play'):
            self.rate.sleep
        return 'game_over'

    def play_callback_ball(self, data):
        global playtime
        if (rospy.get_param('state') == 'play' and rospy.get_param('ball_detected') == 0):
            rospy.loginfo('dog: I have lost the ball :(')
            playtime = 0


def main():
    rospy.init_node('dog_fsm_node')

    sm = smach.StateMachine(outcomes=['container_interface'])
    with sm:
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_play':'PLAY', 
                                            'go_sleep':'SLEEP'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'game_over':'NORMAL'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()


    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
