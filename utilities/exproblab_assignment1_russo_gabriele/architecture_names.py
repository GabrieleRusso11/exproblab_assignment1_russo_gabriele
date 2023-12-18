#!/usr/bin/env python

"""
.. module:: architecture_names
   :platform: Ubuntu 20.04
   :synopsis: Python module which implements a 'dictionary' that defines names and a function for logging in the architecture.

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

it implements a 'dictionary' that defines names and a function for logging used among all the components of the architecture of this assignment.

"""

import rospy

# ---------------------------------------------------------

# The name of the robot state node which represent the shared knowledge.
NODE_ROBOT_STATE = 'robot_state'

# The name of the server to get the actual robot position.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the actual robot position. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# ---------------------------------------------------------

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# -------------------------------------------------

# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# -------------------------------------------------

# the name of the state machine node
NODE_STATEMACHINE = 'state_machine'

# the name of the server used to start the robot charging process
SERVER_CHARGING = 'inteface/start_charging' 


# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return f'@{producer_tag}>> {msg}'
