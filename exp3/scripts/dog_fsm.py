#!/usr/bin/env python

## @package dog_fsm
# Emulates the robotic dog's finite state machine internal architecture. 
# The implemented states are Sleep, Normal, Play and Find

import rospy
import smach
import smach_ros
import time
import random
import actionlib
from std_msgs.msg import String
from exp3.srv import BallService, Explore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

## Acquire parameters from the launch file
map_x_max = rospy.get_param('map/x_max')
map_y_max = rospy.get_param('map/y_max')

map_x_min = rospy.get_param('map/x_min')
map_y_min = rospy.get_param('map/y_min')

home_x = rospy.get_param('home/x')
home_y = rospy.get_param('home/y')

room_list = rospy.get_param('room_list')

sim_scale = rospy.get_param('sim_scale')

## variable used to let the fsm behave differently for the very first state only
first_iteration = 1

## variable used to state whether it is time to play or not
playtime = 0

## variable used to identify and store the requested room/ball
play_ball_request = 100

## variable used to keep track of the robot's simulated battery charge
energy_timer = random.randint(4, 7)



## Sleep state
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up'])

    def execute(self, userdata):
        global first_iteration, energy_timer
        rospy.set_param('state','sleep')
        self.rate = rospy.Rate(200)
        mb_sleep_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if first_iteration == 0:
            home_pos = MoveBaseGoal()
            home_pos.target_pose.pose.orientation.w = 1.0
            home_pos.target_pose.header.frame_id = "map"
            home_pos.target_pose.header.stamp = rospy.Time.now()
            home_pos.target_pose.pose.position.x = home_x
            home_pos.target_pose.pose.position.y = home_y
            rospy.loginfo('Dog: I am going to spleep!')
            mb_sleep_client.send_goal(home_pos)
            while(mb_sleep_client.get_state() != 3):
                self.rate.sleep
            rospy.loginfo('Dog: Home position reached!')
        else:
            mb_sleep_client.wait_for_server()
            first_iteration = 0
        time.sleep(random.randint(5, 10) / sim_scale)
        energy_timer = random.randint(2, 7)
        rospy.loginfo('Dog: Good morning!')
        return 'wake_up'



## Normal state
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'])
        rospy.Subscriber('play_topic', String, self.normal_callback)
    
    def execute(self, userdata):
        global playtime, energy_timer
        rospy.set_param('state', 'normal')
        self.rate = rospy.Rate(200)
        mb_normal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        goal_pos = MoveBaseGoal()
        goal_pos.target_pose.pose.orientation.w = 1.0
        goal_pos.target_pose.header.frame_id = "map"
        while (energy_timer != 0 and playtime == 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'normal'):
            stop_var = 0
            goal_pos.target_pose.header.stamp = rospy.Time.now()
            goal_pos.target_pose.pose.position.x = random.randint(map_x_min, map_x_max)
            goal_pos.target_pose.pose.position.y = random.randint(map_y_min, map_y_max)
            mb_normal_client.send_goal(goal_pos)
            rospy.loginfo('Dog: I am moving to %i %i', \
                 goal_pos.target_pose.pose.position.x, goal_pos.target_pose.pose.position.y)
            while (mb_normal_client.get_state() != 3 and stop_var != 1):
                if(rospy.get_param('new_ball_detected') == 1):
                    rospy.loginfo('Dog: I have spotted a new room!')
                    while(rospy.get_param('new_ball_detected') == 1):
                        stop_var = 1
                        self.rate.sleep
                elif (mb_normal_client.get_state() > 3):
                    rospy.loginfo('Dog: I aborted reaching the last random target')
                    break
                if(playtime == 1):
                    break
                self.rate.sleep
            if(mb_normal_client.get_state() == 3):
                rospy.loginfo('Dog: target position reached!')
            mb_normal_client.cancel_all_goals()
            energy_timer = energy_timer - 1
            self.rate.sleep

        if energy_timer == 0:
            rospy.loginfo('Dog: I am going to sleep!')
            return 'go_sleep'

        elif playtime == 1:
            playtime = 0
            return 'go_play'

    def normal_callback(self, data):
        global playtime
        if (rospy.get_param('state') == 'normal'):
            rospy.loginfo('Dog: I have received a play request! Woof!')
            mb_normal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            mb_normal_client.cancel_all_goals()
            playtime = 1



## Play state
class Play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['game_over','go_find'])
        ## subscribed topic, used to receive commands from the human.py node                    
        rospy.Subscriber('play_topic', String, self.play_callback)

    def execute(self, userdata):
        global play_ball_request, energy_timer
        rospy.set_param('state', 'play')
        self.rate = rospy.Rate(200)
        mb_play_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        target_pos = MoveBaseGoal()
        target_pos.target_pose.pose.orientation.w = 1.0
        target_pos.target_pose.header.frame_id = "map"
        target_pos.target_pose.header.stamp = rospy.Time.now()
        target_pos.target_pose.pose.position.x = home_x
        target_pos.target_pose.pose.position.y = home_y
        rospy.set_param('play_task_status', 0)
        mb_play_client.send_goal(target_pos)
        while(mb_play_client.get_state() != 3):
            self.rate.sleep
        rospy.loginfo('Dog: I reached your position')
        rospy.set_param('play_task_status', 1)
        rospy.wait_for_message('play_topic', String)
        while ((not rospy.is_shutdown()) and energy_timer != 1 \
                        and rospy.get_param('state') == 'play'):
            rospy.set_param('play_task_status', 0)
            while(play_ball_request == 100):
                self.rate.sleep
            rospy.wait_for_service('BallService')
            ball_service_client = rospy.ServiceProxy('BallService', BallService)
            ball_location = ball_service_client(play_ball_request)
            temp_unknown_ball = play_ball_request
            play_ball_request = 100
            if (ball_location.res.x != 100):
                rospy.loginfo('Dog: starting my journey towards the %s', \
                        room_list[temp_unknown_ball])
                target_pos.target_pose.pose.orientation.w = 1.0
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = ball_location.res.x
                target_pos.target_pose.pose.position.y = ball_location.res.y
                mb_play_client.send_goal(target_pos)
                while(mb_play_client.get_state() != 3):
                    self.rate.sleep
                rospy.loginfo('Dog: I got to the room')
                target_pos.target_pose.pose.orientation.w = 1.0
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = home_x
                target_pos.target_pose.pose.position.y = home_y
                mb_play_client.send_goal(target_pos)
                while(mb_play_client.get_state() != 3):
                    self.rate.sleep
                rospy.loginfo('Dog: I am finally back to you')
                rospy.set_param('play_task_status', 2)
                energy_timer = energy_timer - 1
            else:
                rospy.set_param('unknown_ball', temp_unknown_ball)
                break
            self.rate.sleep

        if energy_timer == 1:
            rospy.loginfo('Dog: I am too tired to play any longer: ' + \
                             'I will briefly go in Normal')
            return 'game_over'

        elif rospy.get_param('unknown_ball') != 100:
            rospy.loginfo('Dog: I don\'t know where the %s is. ' + \
                             'I\'ll search around for it', \
                             room_list[rospy.get_param('unknown_ball')])
            return 'go_find'

    ## Play state callback that is used to translate the room ordered
    # by the human into the ball representing that same room, and that will then be
    # reached or searched by the robotic dog
    def play_callback(self, data):
        global play_ball_request
        if (rospy.get_param('state') == 'play' and data.data != 'play'):
            rospy.loginfo('Dog: I will try to go to the %s', data.data)
            if data.data == room_list[0]:
                play_ball_request = 0
            elif data.data == room_list[1]:
                play_ball_request = 1
            elif data.data == room_list[2]:
                play_ball_request = 2
            elif data.data == room_list[3]:
                play_ball_request = 3
            elif data.data == room_list[4]:
                play_ball_request = 4
            else:
                play_ball_request = 5



## Find state
class Find(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['find_over'])

    def execute(self, userdata):
        rospy.set_param('state', 'find')
        self.rate = rospy.Rate(200)
        rospy.wait_for_service('explore_start_service')
        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
        explore_start(1)
        rospy.loginfo('Dog: Exploration started')
        while ((not rospy.is_shutdown()) and rospy.get_param('state') == 'find' \
             and rospy.get_param('unknown_ball') != 100):
            self.rate.sleep
        rospy.loginfo('Dog: I found the room you asked for!')
        rospy.set_param('play_task_status', 2)
        rospy.set_param('unknown_ball', 100)
        return 'find_over'


# Create a SMACH state machine
def main():
    rospy.init_node('dog_fsm_node', anonymous = True)

    sm = smach.StateMachine(outcomes=['container_interface'])

    with sm:

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_play':'PLAY', 
                                            'go_sleep':'SLEEP'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'game_over':'NORMAL',
                                            'go_find':'FIND'})
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'find_over':'PLAY'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
