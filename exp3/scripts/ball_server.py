#!/usr/bin/env python


# Implements a server providing the coordinates of the ball whose color matches the input room.
# The received room must have already been coded in a specific single digit integer format in order to be correctly parsed

import rospy
from exp3.srv import BallService
from exp3.msg import Coordinates

## BallService callback: which sets the corresponding ball color coordinates as response
def match_balls(req):
    res = Coordinates()
    if req == 0:
        res.x = rospy.get_param('blue/x')
        res.y = rospy.get_param('blue/y')
    elif req == 1:
        res.x = rospy.get_param('red/x')
        res.y = rospy.get_param('red/y')
    elif req == 2:
        res.x = rospy.get_param('green/x')
        res.y = rospy.get_param('green/y')
    elif req == 3:
        res.x = rospy.get_param('yellow/x')
        res.y = rospy.get_param('yellow/y')
    elif req == 4:
        res.x = rospy.get_param('magenta/x')
        res.y = rospy.get_param('magenta/y')
    else:
        res.x = rospy.get_param('black/x')
        res.y = rospy.get_param('black/y')
    return res

## BallService initialization
def ball_server():
    rospy.init_node('ball_server_node')
    s = rospy.Service('BallService', BallService, match_balls)
    rospy.spin()

if __name__ == "__main__":
    ball_server()
