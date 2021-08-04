#! /usr/bin/env python

import rospy
import time
import random


from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None
count = 0


def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    take_action(regions)


def take_action(regions):
    global count
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if count != 0:
        if count == 200:
            count = 0
        else:
            count = count+1
    else:
        if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 2 - fleft and fright'
            linear_x = 0.3
            angular_z = 0
        elif (regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1) or (regions['front'] < 1 and regions['fleft'] < 1 and regions['fright']) < 1:
            count = 1
            state_description = 'case 3 - front'
            linear_x = 0
            while (abs(angular_z) < 0.3):
                angular_z = random.uniform(-1.0, 1.0)
        elif regions['fright'] < 1:
            count = 1
            state_description = 'case 4 - fright'
            linear_x = 0
            angular_z = random.uniform(0.3, 1.0)
        elif regions['fleft'] < 1:
            count = 1
            state_description = 'case 5 - fleft'
            linear_x = 0
            angular_z = random.uniform(-1.0, -0.3)
        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        pub.publish(msg)


def main():
    global pub

    rospy.init_node('exploring')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
