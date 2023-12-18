#!/usr/bin/env python

"""
.. module:: planner
  :platform: Ubuntu 20.04
  :synopsis: Python module to evaluate the path in order to reach the target location

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

This node implements an Action server that receives the target location from the State Machine node
and the current robot position from the Robot-state node. Then it computes the path from the current 
position to the target position, and return, as result, the total plan to the Action Client which is 
the State Machine node.

This is a simulated program, therefore the planner does not actually have to compute anything,
it simulates work by busy waiting.

ROS Parameters:
  **/planning_time** the waiting time used to simulate the computation of the plan. \n

Service:
  **/state/get_pose** get from the Robot State node the current robot position.\n

Action Server:
  **/motion/planner** given a target location as a goal, computes the path in order to reach it.\n

"""
import random
import rospy

# Import constant name defined to structure the architecture.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import the simple ActionServer.
from actionlib import SimpleActionServer

# Import custom message, actions and services.
from exproblab_assignment1_russo_gabriele.msg import Location, PlanFeedback, PlanResult
from exproblab_assignment1_russo_gabriele.srv import ActualPosition
import exproblab_assignment1_russo_gabriele  # This is required to pass the `PlanAction` type 
                                             # for instantiating the `SimpleActionServer`.

# to color the text
import colorama
from colorama import Fore

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER

# An action server to simulate the motion planning.

class PlaningAction(object):
    """
    This class represents the planner and its /motion/planner action server
    used to simulate the motion planning. Given a target position, 
    it retrieve the current robot position from the `robot-state` node, 
    and return a plan.
    """

    def __init__(self):
        """
        Gets the planning time, used to simulate the planning computation time,
        from the parameter server, define the action server and starts it.
        """

        # Get the delay used to simulate the planning computation time
        # as ros parameter (from the launch file).
        self.delay = rospy.get_param("/planning_time")

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      exproblab_assignment1_russo_gabriele.msg.PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()

        # Log information.
        log_msg = (Fore.LIGHTYELLOW_EX + f'`{anm.ACTION_PLANNER}` Action Server initialised.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)
      
    
    def execute_callback(self, goal):
        """
        The callback executed when a client set a goal to the `planner` server.
        This function will return a list of Locations (i.e., the plan) which contains
        as the fist point the current robot position (retrieved from the `robot-state` node 
        through the /state/get_pose service), whereas the last point is the `goal` position 
        (given from the state machine as goal for the action server). 
        The plan will contain an unknown intermediate position used to simulate a via point 
        path through the target location. To simulate computation, there is a delay.\n

        Args:
          goal (PlanGoal): the goal location
        
        """
        
        # Get the input parameters to compute the plan, i.e., the start (or current) and target locations.
        start_location = _get_client_position()
        intermediate_position = Location('intermediate position')
        target_location = goal.target

        # Check if the start and target locations are correct. If not, this service will be aborted.
        if start_location is None or target_location is None:
            log_msg = Fore.LIGHTRED_EX + 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            print(Fore.WHITE)

            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting location of the plan.
        feedback = PlanFeedback()
        feedback.plan = []
        feedback.plan.append(start_location)

        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        delay = self.delay
        rospy.sleep(delay)

        # Check that the client did not cancel this service.
        if self._as.is_preempt_requested():
            rospy.loginfo(anm.tag_log(Fore.LIGHTRED_EX + 'Server has been cancelled by the client!', LOG_TAG))
            print(Fore.WHITE)

            # Actually cancel this service.
            self._as.set_preempted()  
            return
        
        # evaluation of an intermediate position between the starting location and the target location
        feedback.plan.append(intermediate_position)

        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        delay = self.delay
        rospy.sleep(delay)

        # Check that the client did not cancel this service.
        if self._as.is_preempt_requested():
            rospy.loginfo(anm.tag_log(Fore.LIGHTRED_EX + 'Server has been cancelled by the client!', LOG_TAG))
            print(Fore.WHITE)

            # Actually cancel this service.
            self._as.set_preempted()  
            return
    
        # Append the target location to the plan as the last element of the list.
        feedback.plan.append(target_location)

        # Publish the results to the client.        
        result = PlanResult()

        # sending to the client the plan to reach the
        # target location (in this case the client is the state machine)
        result.plan = feedback.plan 
        self._as.set_succeeded(result)

        log_msg = Fore.GREEN + 'Motion plan succeeded.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)


def _get_client_position():
    """
    Retrieve the current robot position by the `state/get_pose`
    server of the `robot-state` node.\n

    Returns:
      position (Location): the current robot position
      
    """
    # Eventually, wait for the server to be initialized.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:

        # Call the service and get as response the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, ActualPosition)
        response = service()
        position = response.position

        # Log service response.
        log_msg = Fore.LIGHTYELLOW_EX + f'Retrieving current robot position : ({position.name}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)
        return position
    
    except rospy.ServiceException as e:

        log_msg = Fore.LIGHTRED_EX + f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))




if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()