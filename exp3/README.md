# Assignment 3 Description

### Introduction

The core goal of this project is to allow a mobile robot to move around its world, explore and map (SLAM) it, listen to and respond to specific orders given by the user, and register the locations of different rooms using different coloured balls.


<img width="568" alt="Screenshot 2021-08-05 at 00 47 28" src="https://user-images.githubusercontent.com/23742278/128271087-419e2914-1e8f-4d42-9abb-d9d568a609ed.png">


The world is composed of 6 rooms, that have a colored ball inside of them. Each ball has a different color. Each color is mapped to a specific room name. The world is completely unknown to the robot at the beginning of program execution.

The Robot is similar to the one in the Assignment 2, but this one has a Laser Scanner, so it could scan and create a map in `rviz` as you can see in the following figure.


<img width="1048" alt="Screenshot 2021-08-05 at 02 27 46" src="https://user-images.githubusercontent.com/23742278/128272531-8b55647f-2de3-445c-90d9-4e46017fb060.png">

The computed world map is acquired by reading from the `map topic`, which is published by the gmapping laser-based SLAM algorithm. It uses the laser scan data in order to chart the house walls.

### Architecture

Components of this package:

`human.py`, that is act like the dog's owner, which randomly decides to play with the robotic dog. 

The human checks if the robot can play, then waits for it to approach, then commands it to move to a random room of the home, and then waits for it to return. When the robot becomes tired, it goes to sleep mode.

`dog_fsm.py` which is used to handle the robotic dog's FSM internal architecture (the 4 modes that the robot has)

`dog_vision.py` It is a vision module for the robotic dog that makes use of OpenCV to constantly scan the environment for specified coloured balls. This node can take control of the robot's motions when necessary, allowing it to reach a room when a new ball is discovered. It also saves the positions of the discovered balls, allowing it to understand the displacement of the rooms inside the house over time.

`Ball server.py` It's a server that gives you the coordinates of the ball whose colour matches the one in the input room. In order to be correctly interpreted, the receiving room must have been coded in a certain single digit integer format.

`explore_lite`  it allow the robot to explore the home during the FIND state.

### State Machine


<img width="802" alt="Screenshot 2021-08-05 at 02 49 14" src="https://user-images.githubusercontent.com/23742278/128273979-c43cbb72-0d35-4ae9-bb81-662705034d7e.png">


The robot will behave differently depending of its current state. It can be in four different states:

`SLEEP`:
When the robot returns home, he falls asleep. It uses the move_base algorithm to move around in the environment, and it does so by implementing an action client and asking it to get the coordinates for home.

`NORMAL`:
The robot moving around at random by providing randomly generated positions to the move_base. If the robot detects a new room/ball while moving around, it enters it and records the position. Every newly discovered room, as well as the data collection process that follows, reduces the robot's battery. If the battery runs out, the robot switches to the Sleep state and goes to sleep. 
A play command from the human can be received at any moment, and the robotic dog will shift to the Play mode.

`PLAY`: The robot gets back home after receiving the `PLAY` state using the move_base, and then it begins listening for the room request. Once received, it uses a service in `ball_server.py` to determine which ball coordinates should be reached. If the dog hasn't seen the ball yet, it will begin heading to the designated room, dependent on move_base. It returns to the user after reaching that point. If the battary is consumed, the robot goes to the `NORAML` then with the remaining battary, it goes home position and it sleeps (switched to `SLEEP`)
If the the ball is not recognised yet by the robot in the database, it switches to the `FIND` state.

`FIND`: The robot goes to this state when the user/human ask the robot, in the `PLAY` state, to go to unrecognised ball/room. It uses `explore_lite` to perform the exploration. The exploration algorithm keeps running until dog_vision.py stops its execution. If the new ball is the desired goal, the robot gets back to the Play state.
Another way to get into the `FIND` state is when the robot is in the `NORMAL` state and see unrecognised ball/room.

The available nodes and topics could be found in the following rqt_graph:

![rosgraph](https://user-images.githubusercontent.com/23742278/128281418-13c68e77-746f-4227-b9d5-2f85ceeba43e.png)

#### Topics involved:
* Where the `play_topic` is used by the human node in order to perform the  `PLAY` state, and which room the robot should reach. It also sends data to the finiste state machine node.


#### The message types used are:
* exp3.msg/Coordinates: message made of two integers x and y


#### The services message types used:
* exp3.srv/Explore: a simple `int64` input and output service to tell the explore when to start and end.
* exp3.srv/BallService: used by `dog_fsm.py` in order to ask for the goal ball's coordinates. If the answer (of type Coordintates) is (100, 100), the requested ball is still to be found, the server is `ball_server.py`


### Installation & Running

See the [MAIN](https://github.com/Qaoud/EXP-ROB_ASSIGNMENT_2-3) branch to be able to install and run this package.

-----

author: Mohamed Qaoud - mohamed.qaoud@outlook.com



