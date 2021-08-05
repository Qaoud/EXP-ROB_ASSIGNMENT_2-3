# Assignment 2 Description

### Introduction

The goal of this project is to use gazebo simulations to simulate the movement of the robot in an enviroment. Gazebo include plugins like as sensors and controllers.

It's based on the first assignment, where some of this code has previously been implemented, in [Assignment 1](https://github.com/Qaoud/exp_rob)

<img width="703" alt="Screenshot 2021-08-05 at 00 45 15" src="https://user-images.githubusercontent.com/23742278/128265628-c2ed8c2d-56f8-4765-93b7-1bfe8ee6ba49.png">

In a simulated environment, a wheeled robot moves around with a ball model that moves about randomly. On top of the robot's head sits a simulated camera. When the robot notices the ball, it immediately begins to follow it. When the robot gets close enough to the ball and it isn't moving, it looks to the left, then to the right, and finally to the front. When the robot loses sight of the robot for three seconds, it resumes its random movement. The robot goes to a gazebo to sleep for an unknown amount of time at random times (within specific bounds).

There are three possible states for the robotic dog. The first is `SLEEP`, which occurs after a specific period of time in the `NORMAL` state: the dog will return home and sleep for a while. The robot is in the normal state when it wanders about the playing field. Finally, when the dog detects a ball, the `PLAY` state is triggered, and the dog begins to follow it. To avoid colliding with the sphere, the robo-dog will change its distance from it. When the dog notices the ball is still, it moves its head from left to right before returning to pursuing the ball. Because humans can hide the ball from time to time, if the robot loses track of the ball for a long enough period of time, it will revert to its previous state.

### Architecture

THere are five main components of the package, which are:

`human.py`: We can say that this a real human that controls the ball movement, whether it's in the play ground and the robot could see it or hide it from the robot by set the z-axis to negative value.

`go_to_point_ball.py`: this recieve the position of the ball from `human.by` and move/hide the ball in teh playground

`dog_control_ball.py`: this makes the robot follow the ball or look for it. it also move the robot's head when it reaches the ball.

`dog_control.py`: this gives the robot random point to go to when it's in the `NORMAL` state. Also, it moves the robot to home when it's in the `SLEEP` mode.

`dog_fsm.py` which is used to handle the robotic dog's FSM internal architecture (the 3 modes that the robot has)

### State Machine

The dog starts in the Sleep state.

<img width="750" alt="Screenshot 2021-08-05 at 01 30 23" src="https://user-images.githubusercontent.com/23742278/128268471-76f55aa8-2fff-4ec0-8bda-20181a083264.png">

The available nodes and topics could be found in the following rqt_graph:

![rosgraph_exp2](https://user-images.githubusercontent.com/23742278/128283114-c10bff60-a6d6-4c53-97e1-2f2519fc027e.png)


Topics available:

`control_topic`: topic used by the FSM to order the Dog_control component to start simulating a movement
`motion_over_topic`: topic whose duty is to inform the FSM when the motion is over or interrupted by the sight of the ball
`ball_control_topic`: topic used by Dog control ball to communicate with Dog FSM: it sends information when the ball is first spotted by the robot, and then when the dog eventually loses track of it

The actions used:

`exp2.msg/PlanningAction`: a simple action whose goals are of type `geometry_msgs/PoseStamped`

The message types:

`std_msgs.msg/Int64`: imported message type consisting in an integer
`exp2.msg/Coordinates`: message made of two integers x and y

### Installation $ Running

See the [MAIN](https://github.com/Qaoud/EXP-ROB_ASSIGNMENT_2-3) branch to be able to install and run this package.

### Limitation

* I think if there is an obstacle avoindance the robot could operate more accurate and avoid crashes to the human model.
* In my case, as running the whole package on a virtual machine, was a bit slow and it might cause some issues when running the package

### Author
Mohamed Qaoud - mohamed.qaoud@outlook.com



