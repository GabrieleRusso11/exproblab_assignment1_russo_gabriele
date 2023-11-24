#!/usr/bin/env python

import random
import rospy

# Import constant name defined to structure the architecture.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import the simple ActionServer.
from actionlib import SimpleActionServer

# Import custom message, actions and services.
from exproblab_assignment1_russo_gabriele.msg import ControlFeedback, ControlResult
from exproblab_assignment1_russo_gabriele.srv import NewPosition
import exproblab_assignment1_russo_gabriele  # This is required to pass the `ControlAction` type for 
                                             # instantiating the `SimpleActionServer`.

# used to color the output text
import colorama
from colorama import Fore

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER


# An action server to simulate motion controlling.
# Given a plan as a list of locations, it simulate the movements
# to reach each location with a delay. This server updates
# the current robot position stored in the `robot-state` node.
class ControllingAction(object):

    def __init__(self):
        # Get the delay ros parameter used by this server
        # to simulate the movements
        self.delay = rospy.get_param("/moving_time")

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      exproblab_assignment1_russo_gabriele.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        # Log information.
        log_msg = (Fore.LIGHTCYAN_EX + f'`{anm.ACTION_CONTROLLER}` Action Server initialised.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in 
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):
        
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.plan is None or len(goal.plan) == 0:
            rospy.logerr(anm.tag_log(Fore.LIGHTRED_EX + 'No locations provided! This service will be aborted!', LOG_TAG))
            print(Fore.WHITE)
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for location in goal.plan:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log(Fore.LIGHTRED_EX + 'Service has been cancelled by the client!', LOG_TAG))
                print(Fore.WHITE)
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            delay = self.delay
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_location = location
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            _set_client_position(location)
            # Log current robot position.
            log_msg = Fore.LIGHTCYAN_EX + f'Reaching location ({location.name}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print(Fore.WHITE)

        # Publish the results to the client.
        result = ControlResult()
        result.final_location = feedback.reached_location
        log_msg = Fore.CYAN + 'Motion control successes.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)
        self._as.set_succeeded(result)
        return  # Succeeded.


# Update the current robot `pose` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
def _set_client_position(position):
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Log service call.
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, NewPosition)
        service(position)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = Fore.LIGHTRED_EX + f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()