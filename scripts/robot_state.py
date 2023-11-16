#!/usr/bin/env python

import threading
import random
import rospy
# Import constant name defined to structure the architecture.
import architecture_names as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from exproblab_assignment1_russo_gabriele.srv import ActualPosition, ActualPositionResponse, NewPosition, NewPositionResponse


def tag_log(msg, producer_tag):
    return f'@{producer_tag}>> {msg}'
# A tag for identifying logs producer.
NODE_ROBOT_STATE = 'robot-state'
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'
# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

LOG_TAG = NODE_ROBOT_STATE

# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    def __init__(self):
        # Initialise this node.
        rospy.init_node(NODE_ROBOT_STATE, log_level=rospy.INFO)

        # Initialise robot position.
        self._position = None

        # Initialise battery level.
        self._battery_low = False

        # Initialise randomness, if enabled.
        self._random_battery_time = rospy.get_param(PARAM_BATTERY_TIME, [15.0, 40.0])
        log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                    f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
        rospy.loginfo(tag_log(log_msg, LOG_TAG))

        # Define services.
        rospy.Service(SERVER_GET_POSE, ActualPosition, self.get_position)
        rospy.Service(SERVER_SET_POSE, NewPosition, self.set_position)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()
        
        # Log information.
        log_msg = (f'Initialise node `{NODE_ROBOT_STATE}` with services `{SERVER_GET_POSE}` and '
                   f'`{SERVER_SET_POSE}`, and topic {TOPIC_BATTERY_LOW}.')
        rospy.loginfo(tag_log(log_msg, LOG_TAG))

    # The `robot/set_pose` service implementation.
    # The `request` input parameter is the current robot pose to be set,
    # as given by the client. This server returns an empty `response`.
    def set_position(self, request):
        if request.position is not None:
            # Store the new current robot position.
            self._position = request.position
            # Log information.
            self._print_info(f'Set current robot position through `{SERVER_SET_POSE}` '
                             f'as ({self._position}).')
        else:
            rospy.logerr(tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return NewPositionResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty. Thus, it is not used.
    # The `response` returned to the client contains the current robot pose.
    def get_position(self, request):
        # Log information.
        if self._position is None:
            rospy.logerr(tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = f'Get current robot position through `{SERVER_GET_POSE}` as ({self._position})'
            self._print_info(log_msg)
        # Create the response with the robot pose and return it.
        response = ActualPositionResponse()
        response.position = self._position
        return response

    # Publish changes of battery levels. This method runs on a separate thread.
    # Publish when the battery change state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def _is_battery_low(self):
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                log_msg = f'Robot got low battery after {delay} seconds.'
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
            self._print_info(log_msg)
            # Wait for simulate battery usage.
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low

    # Print logging only when random testing is active.
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def _print_info(self, msg):
        
        rospy.loginfo(tag_log(msg, LOG_TAG))


if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()