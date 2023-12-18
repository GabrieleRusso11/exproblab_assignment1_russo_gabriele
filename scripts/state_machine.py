#!/usr/bin/env python

"""
.. module:: state_machine       
   :platform: Ubuntu 20.04
   :synopsis: Python module which implements the Final State Machine of the architecture.

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

This module manages the Finite State Machine (FSM) of the architecture, 
which is implemented by using The `ROS SMACH library <http://wiki.ros.org/smach>`_.
States are non-concurrent and updated at a rate defined by the LOOP_SLEEP_TIME global variable.

ROS Parameters:
  **/waiting_time** the waiting time used to simulate that the robot does something when is in the reached location. \n

Service:
  **/state/set_pose** sends the initial robot position to the robot state node .\n

"""

import sys
import random

# Import a class that is an interface of the state machine, used to handle 
# the comunication with the other nodes but also to contains function used by
# the state machine to compute data, for instance the creation of the ontology
# and the retrieving of information from the ontology itself.
from exproblab_assignment1_russo_gabriele.state_machine_interface import Handler

# Imports relative to ROS and the SMACH library.
import rospy
import smach_ros
from smach import StateMachine, State

# Import constant names that define the architecture's structure.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import custom message, actions and services.
from exproblab_assignment1_russo_gabriele.msg import Location, PlanAction, PlanGoal, ControlAction, ControlGoal, ControlResult
from exproblab_assignment1_russo_gabriele.srv import NewPosition

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 1

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATEMACHINE

# to color the text
import colorama
from colorama import Fore

# The list of names that identify the states of the Finite State Machine.
STATE_RECHARGING = 'RECHARGING'             # The name of the state where the robot waits to recharging its battery.
STATE_WAIT_FOR_TOP_MAP = 'WAIT_FOR_TOP_MAP' # The name of the initial state where the robot waits in the room E for the topological map.
STATE_WAIT = 'WAIT'                         # The name of the state where the robot waits in the room after reached it.
STATE_PLANNING = 'PLANNING'                 # The name of the state where the robot plans what is the next location to visit.
STATE_MOVING = 'MOVING'                     # The name of the state where the robot moves toward a location.

# The list of names that identify the transitions of the Finite State Machine.
TRANS_BATTERY_LOW = 'battery_low'                    # The transition from the 'PLANNING', 'MOVING' and 'WAIT' states toward the `RECHARGING` state.
TRANS_CHARGED_BATTERY = 'charged_battery'            # The transition from the 'RECHARGING' state to the 'PLANNING' state.
TRANS_NO_TOP_MAP = 'no_top_map'                      # The transition from the 'WAIT_FOR_TOP_MAP' to itself.
TRANS_TOP_MAP_READY = 'top_map_ready'                # The transition from the 'WAIT_FOR_TOP_MAP' to 'PLANNING' state. 
TRANS_IN_A_ROOM = 'in_a_room'                        # The transition from the 'MOVING' to 'WAIT' state.
TRANS_TIMEOUT = 'timeout'                            # The transition from the 'WAIT' to 'PLANNING' state.
TRANS_MOVING = 'moving'                              # The transition from the 'PLANNING' to 'MOVING' state.
TRANS_SEARCH_CHARG_LOC = 'search_charging_location'  # The transition from the 'RECHARGING' to 'PLANNING' state.

# variable passed from the PLANNING state to the MOVING state
VAR_PLAN = 'plan'  

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3

class WaitForTopMap(State):
    """
    The definition of the wait_for_top_map state.
    It is the state where the robot waits that the 
    map (creation of the ontology) is ready.
    """
    # Construct this class, i.e., initialize this state.
    def __init__(self, state_machine_interface):

        # Get a reference to the interface with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialize this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_NO_TOP_MAP, TRANS_TOP_MAP_READY])

    # Define the function performed each time the state machine enter in this state,
    # thanks to a proper transition.
    def execute(self, userdata):
        """
        Function performed each time the state machine enter in the Wait_For_Top_Map state.
        It creates the map and sets all the important parameter of the ontology.
        Once the map is ready, it sends the starting position to the robot state node (through the /state/set_pose)
        and goes in the next state (Planning state) through the 'top_map_ready' transition, otherwise,
        it continues to wait that the map is ready ('no_top_map' transition).
        """
         # Wait the server to set the starting robot position
        # in the robot state node
        rospy.wait_for_service(anm.SERVER_SET_POSE)

        # method of the class Handler that create the map and set
        # all the important parameter of the ontology
        self._interface.set_map()

        # send the starting robot position to the robot state node.
        start_pos = Location(self._interface._init_pos)
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, NewPosition)
        service(start_pos)  # The `response` is not used.
        
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  

            # Acquire the mutex to assure data consistencies 
            # with the ROS subscription threads managed by `self._interface`.
            self._interface.mutex.acquire()

            try:

                # if the map is ready go in the planning state, 
                # through the TRANS_TOP_MAP_READY
                if self._interface.end:

                    print(Fore.WHITE)
                    print(Fore.YELLOW + "the map is ready")
                    print(Fore.WHITE)

                    return TRANS_TOP_MAP_READY
                
                else:

                    print(Fore.WHITE)
                    print(Fore.YELLOW + "the map is not ready")
                    print(Fore.WHITE)

                    return TRANS_NO_TOP_MAP
                
            finally:

                # Release the mutex to unblock the `self._interface` 
                # subscription threads if they are waiting.
                self._interface.mutex.release()

            # Wait for a reasonably small amount of time to allow 
            # `self._interface` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class Planning(State):
    """
    The definition of the Planning state.
    It is the state where the robot plans 
    the next position that has to be reached
    """
    # Construct this class, i.e., initialize this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interface with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialize this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_MOVING, TRANS_CHARGED_BATTERY, TRANS_SEARCH_CHARG_LOC,
                                       TRANS_TOP_MAP_READY, TRANS_TIMEOUT,TRANS_BATTERY_LOW], output_keys=[VAR_PLAN])

    # Define the function performed each time the state machine enter in this state,
    # thanks to a proper transition.
    def execute(self, userdata):
        """
        Function performed each time the state machine enter in the Planning state.
        It checks if the robot is in the low battery mode, if the low battery mode is active
        the planning state plans how to reach the charging location, otherwise, it plans how to reach
        the next location (which can be an urgent location and/or a corridor).
        The low battery mode is active when the robot has a low battery level and it is not in the
        charging location when the battery gets low level.
        Once the plan is ready, the planning state, using the Handler class as interface, sends 
        the target location as goal to the Action server (the planner node).
        If during this process, the battery gets low level and the robot is not in the low battery mode
        the goal (the next location to reach) is cancelled and there is the transition from the planning to recharging
        state (low_battery transition).
        Instead, if the battery level is not low or if the robot is in the low battery mode, the state waits that
        the action server (planner node) has finished and so, that the plan is ready.
        Then the plan is sent to the moving state using the 'userdata' and lastly there is the transition from
        the planning state to the moving state.
        """

        # disjoint all the individuals
        self._interface.disjoint()

        # activate the reasoner 
        self._interface.reasoner()

        # get the actual robot position
        actual_pos = self._interface.ask_rob_pos()
    
        print(Fore.LIGHTBLUE_EX + "The current robot position is :", Fore.WHITE)
        print(str(actual_pos))
        print()

        goal = PlanGoal()

        # if the the robot is in the low battery mode, 
        # then search the charging location, otherwise 
        # compute the target location that the robot has to reach
        if self._interface.low_battery_mode:

            goal.target = self._interface.search_charging_location()

        else:
            
            goal.target = self._interface.target_location()

        # send the goal to the planner action server.
        self._interface.planner_client.send_goal(goal)

        print(Fore.LIGHTMAGENTA_EX)
        rospy.loginfo(anm.tag_log('Planning to go in a new location...', LOG_TAG))
        print(Fore.WHITE)

        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  

            # Acquire the mutex to assure data consistencies with the 
            # ROS subscription threads managed by `self._interface`.
            self._interface.mutex.acquire()

            try:

                # if the robot battery is low and the robot is not yet in 
                # the low battery mode, cancel the actual goal and go in the 
                # Recharging state thanks to the TRANS_BATTERY_LOW
                if self._interface.is_battery_low() and not self._interface.low_battery_mode:  
                    
                    # cancel the goal
                    self._interface.planner_client.cancel_goals()

                    return TRANS_BATTERY_LOW
                
                # if the robot has no low battery level and the planner node 
                # has finished to evaluate the plan, then take the plan to 
                # pass to the Moving state and go to the Moving state 
                # thanks to the TRANS_MOVING
                if self._interface.planner_client.is_done():

                    userdata.plan = self._interface.planner_client.get_results().plan [1:]

                    return TRANS_MOVING
                
            finally:

                # Release the mutex to unblock the `self._interface` 
                # subscription threads if they are waiting.
                self._interface.mutex.release()

            # Wait for a reasonably small amount of time to allow
            # `self._interface` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class Moving(State):
    """
    The definition of the Moving state.
    In this state the robot goes from the current location
    to the target location, thanks to the Controller node.
    """

    # Construct this class, i.e., initialize this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interface with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialize this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_BATTERY_LOW, TRANS_IN_A_ROOM, 
                                       TRANS_MOVING], input_keys=[VAR_PLAN])

    # Define the function performed each time the state machine enter in this state,
    # thanks to a proper transition.
    def execute(self, userdata):
        """
        Function performed each time the state machine enter in the Moving state.
        The Moving state receives from the Planning state the variable plan which
        is the target location to reach. This target location is sent to the controller
        node (the action server) using the method of the class Handler.
        If the battery gets a low level and the robot is not in the low battery mode
        the goal (the plan for reach the target location) is cancelled and there is 
        the transition from the moving to recharging state (low_battery transition).
        Instead, if the battery level is not low or if the robot is in the low battery mode,
        the state waits that the controller action server is done and after that, it retrieves
        the new location, as Action result, and then updates the robot position and the robot and
        location timestamp.
        Lastly, if the battery is low, the state machine goes in the Recharging state to check if
        the controller moves the robot in the charging location (and charge the robot), otherwise
        the state machine goes in the 'Wait' state which simulate that the robot does something in the location.
        """

        # get thee current robot position that will be the 
        # old location once the robot starts to moving
        old_location = self._interface.ask_rob_pos()

        # Start the action server for moving the robot through the planned positions.
        control_goal = ControlGoal()

        # plan given by the Planning state
        control_goal.plan = userdata.plan

        # send the goal to the controller node
        self._interface.controller_client.send_goal(control_goal)

        print(Fore.LIGHTMAGENTA_EX)
        rospy.loginfo(anm.tag_log('Following the plan to reach the target location...', LOG_TAG))
        print(Fore.WHITE)

        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  

            # Acquire the mutex to assure data consistencies with the
            # ROS subscription threads managed by `self._interface`.
            self._interface.mutex.acquire()

            try:

                # if the robot battery is low and the robot is not yet in 
                # the low battery mode, cancel the actual goal and go in the 
                # Recharging state thanks to the TRANS_BATTERY_LOW
                if self._interface.is_battery_low() and not self._interface.low_battery_mode:  
                    
                    # cancel the actual goal
                    self._interface.controller_client.cancel_goals()

                    return TRANS_BATTERY_LOW

                # if the robot has no low battery level and the controller node 
                # has finished to move the robot from one location to another, 
                # then take the new position tahnks to the get_results function
                # and updates the robot time and position and also the visited 
                # location time, then go in the Wait state thanks to the TRANS_IN_A_ROOM
                # if the battery level is not low. otherwise go in the Recharging state
                if self._interface.controller_client.is_done():

                    result = self._interface.controller_client.get_results()
                    new_location = result.final_location.name

                    self._interface.update_robot_position(str(old_location),str(new_location))
                    self._interface.update_robot_time()
                    self._interface.update_location_time(str(new_location))

                    # activate the reasoner 
                    self._interface.reasoner()

                    if self._interface.is_battery_low():

                        return TRANS_BATTERY_LOW
                    
                    else:

                        return TRANS_IN_A_ROOM
                    
            finally:

                # Release the mutex to unblock the `self._interface` 
                # subscription threads if they are waiting.
                self._interface.mutex.release()

            # Wait for a reasonably small amount of time to allow `self._interface` 
            # processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

# The definition of the recharging state.
# It is the state where the robot recharges its battery
# after it has reached the charging location
class Recharging(State):
    """
    The definition of the recharging state.
    It is the state where the robot recharges its battery
    after it has reached the charging location.
    """

    # Construct this class, i.e., initialize this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interface with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialize this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_BATTERY_LOW, TRANS_CHARGED_BATTERY, 
                                       TRANS_SEARCH_CHARG_LOC])

    

    # Define the function performed each time the state machine enter in this state,
    # thanks to a proper transition.
    def execute(self, userdata):
        """
        Function performed each time the state machine enter in the Recharging state.
        This state checks if the robot is in the charging location. In case the robot is
        in the charging location It sends to the Robot State node, the boolean flag 'found'
        (True value) through the /interafce/start_charging service, in order to say to the robot state
        node that the charging process can begin because the robot is in the charging location.
        otherwise, if the robot is not in the charging location, activate the low battery mode. 
        This is a mode that sets the Planning and the Moving state in order to search and reach only the
        charging location.
        Lastly, if the battery is recharged, the state variables (and flags) are reset and the found flag 
        is sent again to the robot state node (in order to reset also there the flag) then the state machine 
        comes back to the Planning node, through the 'charged_battery' transition.
        Instead, if the battery is still low and the charging location is not yet found (found flag is False)
        then the state machine goes in the Planning mode, through the 'search_charging_location' transition, 
        in order to compute a plan to reach the charging location.
        """
        # get the current robot position
        robot_is = self._interface.ask_rob_pos()

        # if the robot is in the charging location 
        # then set the found flag to true and send it to the
        # robot state node, in order to start the charging process
        # (the change of state of the battery level managed in the robot state node)
        if robot_is == self._interface.charging_loc:

            self._interface.found = True
            self._interface.set_starting_flag(self._interface.found)

            print(Fore.WHITE)
            print(Fore.LIGHTGREEN_EX + "The Robot is in the Charging Location.")
            print()
            print(Fore.LIGHTGREEN_EX + "...Wait for charging...")
            print(Fore.WHITE)            

        else: # if the robot is not in the charging location,
              # then activate the low battery mode, 
              # and starts to search the charging location

            print(Fore.WHITE)
            print(Fore.LIGHTCYAN_EX + "Searching the charging location mode...")
            print(Fore.WHITE)

            # this flag is used to set the planning and 
            # moving states in the low battery mode and 
            # plan and move only toward the robot charging location
            self._interface.low_battery_mode = True

        # Wait for stimulus from the other nodes of the architecture. 
        while not rospy.is_shutdown():  

            # Acquire the mutex to assure data consistencies with 
            # the ROS subscription threads managed by `self._interface`.
            self._interface.mutex.acquire()

            try:

                # If the battery level is not low anymore, resets all 
                # the states variable (_battery_low, found, low_battery_mode)
                # and send the updated found flag to rhe robot state node.
                # Then return to the Planning state thanks to the TRANS_CHARGED_BATTERY
                if not self._interface.is_battery_low():

                    print()
                    print(Fore.LIGHTCYAN_EX + "The battery is charged.")
                    print()

                    self._interface.reset_states()
                    self._interface.set_starting_flag(self._interface.found)

                    return TRANS_CHARGED_BATTERY
                
                # if the battery level is low and the robot has not 
                # yet reached the charging location, return to 
                # the Planning in order to search the charging location
                # and compute a proper plan
                if self._interface.is_battery_low() and not self._interface.found:

                    return TRANS_SEARCH_CHARG_LOC
                
            finally:

                # Release the mutex to unblock the `self._interface` 
                # subscription threads if they are waiting.
                self._interface.mutex.release()

            # Wait for a reasonably small amount of time to allow
            # `self._interface` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

class Wait(State):
    """
    The definition of the wait state.
    It is the state where is simulated that the robot 
    is doing something in the reached location
    """

    # Construct this class, i.e., initialize this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interface with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialize this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_TIMEOUT, TRANS_BATTERY_LOW, TRANS_IN_A_ROOM])

    # Define the function performed each time the state machine enter in this state,
    # thanks to a proper transition.
    def execute(self, userdata):
        """
        Function performed each time the state machine enter in the Wait state.
        If the battery is not low, this state simply use a busy wait to simulate that the
        robot is doing something, after this waiting time the state machine goes back to
        the Planning state.
        if the battery is low, the state machine goes in the Recharging state
        """
        # get the waiting time from the ros parameter server
        waiting_time = rospy.get_param("/waiting_time")

        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  

            # Acquire the mutex to assure data consistencies with 
            # the ROS subscription threads managed by `self._interface`.
            self._interface.mutex.acquire()

            try:

                # If the battery level is low, go in the 
                # Recharging state thanks to the TRANS_BATTERY_LOW.
                # Otherwise, simulate that the robot is doing something
                # using a waiting time. Once this waiting time is expired
                # go in the Planning state through the TRANS_TIMEOUT
                if self._interface.is_battery_low():

                    return TRANS_BATTERY_LOW
                
                else:

                    print(Fore.WHITE)
                    print(Fore.CYAN + "...WAITING...")
                    print(Fore.WHITE)

                    rospy.sleep(waiting_time)

                    return TRANS_TIMEOUT
                
            finally:

                # Release the mutex to unblock the `self._interface` 
                # subscription threads if they are waiting.
                self._interface.mutex.release()

            # Wait for a reasonably small amount of time to allow 
            # `self._interface` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


def main():
    """
    This is the main function, in which the state machine is initialised,
    configured and run. The Introspection Server is also started to allow the
    state machine to be visualised via the `ROS SMACH library <http://wiki.ros.org/smach>`_.
    """

    # Initialize this ROS node.
    rospy.init_node('state_machine_russo_gabriele')

    # Initialize an interface class to manage the 
    # interfaces with the other nodes in the architectures.
    interface = Handler()

    # Define the structure of the Finite State Machine.
    sm = StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    with sm:

        # Define the inner state to create, set and load the map.
        StateMachine.add(STATE_WAIT_FOR_TOP_MAP, WaitForTopMap(interface),
                            transitions={TRANS_NO_TOP_MAP: STATE_WAIT_FOR_TOP_MAP,
                                        TRANS_TOP_MAP_READY: STATE_PLANNING})

        # Define the inner state to plan the next position.
        StateMachine.add(STATE_PLANNING, Planning(interface),
                            transitions={TRANS_TOP_MAP_READY: STATE_PLANNING,
                                        TRANS_MOVING: STATE_MOVING,
                                        TRANS_CHARGED_BATTERY: STATE_PLANNING,
                                        TRANS_BATTERY_LOW: STATE_RECHARGING,
                                        TRANS_SEARCH_CHARG_LOC: STATE_PLANNING,
                                        TRANS_TIMEOUT: STATE_PLANNING})
        
        # Define the inner state to move to the target location.
        StateMachine.add(STATE_MOVING, Moving(interface),
                            transitions={TRANS_BATTERY_LOW: STATE_RECHARGING,
                                        TRANS_MOVING: STATE_MOVING,
                                        TRANS_IN_A_ROOM: STATE_WAIT})

        # Define the inner state to simulate that the robot is doing something.
        StateMachine.add(STATE_WAIT, Wait(interface),
                            transitions={TRANS_TIMEOUT: STATE_PLANNING,
                                        TRANS_BATTERY_LOW: STATE_RECHARGING,
                                        TRANS_IN_A_ROOM: STATE_WAIT})

        # Define the inner state to recharge the robot battery.
        StateMachine.add(STATE_RECHARGING, Recharging(interface),
                            transitions={TRANS_CHARGED_BATTERY: STATE_PLANNING,
                                         TRANS_BATTERY_LOW: STATE_RECHARGING,
                                         TRANS_SEARCH_CHARG_LOC: STATE_PLANNING})

    # Create and start the introspection server for visualizing the finite state machine.
    intro_server = smach_ros.IntrospectionServer('intro_server', sm, '/SM_ROOT')
    intro_server.start()

    # Execute the state machine. Note that the `outcome` value of the main Finite State Machine is not used.
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    intro_server.stop()


# The function that get executed at start time.
if __name__ == '__main__':
    main()  # Initialize and start the ROS node.
