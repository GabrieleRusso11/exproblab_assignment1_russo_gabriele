#!/usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
from exproblab_assignment1_russo_gabriele import architecture_names as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from exproblab_assignment1_russo_gabriele.msg import Location, Point, PlanFeedback, PlanResult
from exproblab_assignment1_russo_gabriele.srv import ActualPosition
import exproblab_assignment1_russo_gabriele  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER


# An action server to simulate motion planning.
# Given a target position, it retrieve the current robot position from the 
# `robot-state` node, and return a plan as a set of via points.
class PlaningAction(object):

    def __init__(self):
        # Get random-based parameters used by this server
        #self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 1])
        #self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      exproblab_assignment1_russo_gabriele.msg.PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised.')
      
    # The callback invoked when a client set a goal to the `planner` server.
    # This function will return a list of random points (i.e., the plan) when the fist point
    # is the current robot position (retrieved from the `robot-state` node), while the last 
    # point is the `goal` position (given as input parameter). The plan will contain 
    # a random number of other points, which spans in the range 
    # [`self._random_plan_points[0]`, `self._random_plan_points[1]`). To simulate computation,
    # each point is added to the plan with a random delay spanning in the range 
    # [`self._random_plan_time[0]`, `self._random_plan_time[1]`).
    def execute_callback(self, goal):
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_location = _get_pose_client()
        target_location = goal.target

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if start_location is None or target_location is None:
            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.plan = []
        feedback.plan.append(start_location)

        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
        rospy.sleep(delay)
        # Check that the client did not cancel this service.
        if self._as.is_preempt_requested():
            rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))
            # Actually cancel this service.
            self._as.set_preempted()  
            return
    
        # Append the target point to the plan as the last point.
        feedback.via_points.append(target_location)

        # Publish the results to the client.        
        result = PlanResult()
        result.plan = feedback.plan
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

# Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
def _get_pose_client():
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, ActualPosition)
        response = service()
        position = response.position
        # Log service response.
        log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({position}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return position
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()