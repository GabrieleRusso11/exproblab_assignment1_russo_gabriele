#!/usr/bin/env python

# Import a class that decouples the interface of the Finite State Machine with
# the other  nodes of the architecture from the actual implementation of the
# Finite State Machine, which is available in this file.
import sys
from exproblab_assignment1_russo_gabriele.state_machine_interface import Handler

# Use randomness for testing purposes.
import random

# Imports relative to ROS and the SMACH library.
import rospy
import smach_ros
from smach import StateMachine, State

# Import constant names that define the architecture's structure.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import used messages defined within the ROS architecture.
from exproblab_assignment1_russo_gabriele.msg import Location, PlanAction, PlanGoal, ControlAction, ControlGoal, ControlResult
from exproblab_assignment1_russo_gabriele.srv import NewPosition

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATEMACHINE


# Use randomness for testing purposes.
import random

# Imports relative to ROS and the SMACH library.
import rospy
import smach_ros
from smach import StateMachine, State



# to color the text
import colorama
from colorama import Fore

# The list of names that identify the states of the Finite State Machine.
STATE_RECHARGING = 'RECHARGING'             # The name of the state where the robot waits to recharging its battery.
STATE_GO_INTO_A_ROOM = 'GO_INTO_A_ROOM'     # The name of the state where the robot moves toward a room
STATE_WAIT_FOR_TOP_MAP = 'WAIT_FOR_TOP_MAP' # The name of the initial state where the robot waits in the room E for the topological map
STATE_WAIT = 'WAIT'                         # The name of the state where the robot waits in the room after reached it
STATE_PLANNING = 'PLANNING'
STATE_MOVING = 'MOVING'

# The list of names that identify the transitions of the Finite State Machine.
TRANS_BATTERY_LOW = 'battery_low'           # The transition from the 'GO_INTO_A_ROOM' and 'WAIT' states toward the `RECHARGING` state.
TRANS_CHARGED_BATTERY = 'charged_battery'   # The transition from the `RECHARGING` state to the 'GO_INTO_A_ROOM' state.
TRANS_NO_TOP_MAP = 'no_top_map'             # The transition from the 'WAIT_FOR_TOP_MAP' to itself
TRANS_TOP_MAP_READY = 'top_map_ready'       # The transition from the 'WAIT_FOR_TOP_MAP' to 'GO_INTO_A_ROOM' 
TRANS_IN_A_ROOM = 'in_a_room'               # The transition from the 'GO_INTO_A_ROOM' to 'WAIT'
TRANS_TIMEOUT = 'timeout'                   # The transition from the 'WAIT' to 'GO_INTO_A_ROOM'
TRANS_MOVING = 'moving'
TRANS_SEARCH_CHARG_LOC = 'search_charging_location'

VAR_PLAN = 'plan'
VAR_CHARG_FEEDBACK = 'feedback' # boolean value that the planning state 
                                # uses to comunicate to the Recharging state that 
                                # the charging location is reached.

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3

# The definition of the wait_for_top_map state.
# It is the state where the robot waits the the map (creation of the ontology) is ready
class WaitForTopMap(State):

    # Construct this class, i.e., initialise this state.
    def __init__(self, state_machine_interface):

        # Get a reference to the interfaces with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_NO_TOP_MAP, TRANS_TOP_MAP_READY])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
         while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._interface.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if self._interface.is_battery_low():
                    return TRANS_BATTERY_LOW
                else:
                    rospy.wait_for_service(anm.SERVER_SET_POSE)
                    self._interface.set_map()
                    start_pos = Location(self._interface._init_pos)
                    service = rospy.ServiceProxy(anm.SERVER_SET_POSE, NewPosition)
                    service(start_pos)  # The `response` is not used.

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
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._interface.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)



# The definition of the go_into_a_room state.
# It is the state where the robot goes toward a location (the urgent location that needs to be visited)
class Planning(State):

    # Construct this class, i.e., initialise this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interfaces with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_MOVING, TRANS_CHARGED_BATTERY, TRANS_SEARCH_CHARG_LOC,
                                       TRANS_TOP_MAP_READY, TRANS_TIMEOUT,TRANS_BATTERY_LOW], output_keys=[VAR_PLAN,VAR_CHARG_FEEDBACK])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Define a random point to be reached through some via-points to be planned.
        # disjoint all the individuals
        self._interface.disjoint()

        # activate the reasoner 
        self._interface.reasoner()

        actual_pos = self._interface.ask_rob_pos()
    
        print(Fore.LIGHTCYAN_EX + "robot position is :", Fore.WHITE)
        print(str(actual_pos))
        print()

        goal = PlanGoal()

        if self._interface.low_battery_mode:
            goal.target = self._interface.search_charging_location()
        else:
            goal.target = self._interface.target_location()

        # Invoke the planner action server.
        self._interface.planner_client.send_goal(goal)

        print(Fore.LIGHTBLUE_EX)
        rospy.loginfo(anm.tag_log('Planning to go in a new random position...', LOG_TAG))
        print(Fore.WHITE)


        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._interface.mutex.acquire()
            try:
                if self._interface.is_battery_low() and not self._interface.low_battery_mode:  # Higher priority.
                    self._interface.planner_client.cancel_goals()
                    return TRANS_BATTERY_LOW
                if self._interface.planner_client.is_done():
                    userdata.plan = self._interface.planner_client.get_results().plan [1:]
                    return TRANS_MOVING
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._interface.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

# The definition of the go_into_a_room state.
# It is the state where the robot goes toward a location (the urgent location that needs to be visited)
class Moving(State):

    # Construct this class, i.e., initialise this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interfaces with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_BATTERY_LOW, TRANS_IN_A_ROOM, 
                                       TRANS_MOVING], input_keys=[VAR_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):

        old_location = self._interface.ask_rob_pos()
        print()
        print("old location: ")
        print(old_location)

        # Start the action server for moving the robot through the planned via-points.
        control_goal = ControlGoal()
        control_goal.plan = userdata.plan
        self._interface.controller_client.send_goal(control_goal)

        print(Fore.LIGHTRED_EX)
        rospy.loginfo(anm.tag_log('Following the plan to reach the target location...', LOG_TAG))
        print(Fore.WHITE)

        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._interface.mutex.acquire()
            try:

                if self._interface.is_battery_low() and not self._interface.low_battery_mode:  # Higher priority.
                    self._interface.controller_client.cancel_goals()
                    return TRANS_BATTERY_LOW

                if self._interface.controller_client.is_done():
                    result = self._interface.controller_client.get_results()
                    new_location = result.final_location.name
                    print("new position")
                    print(new_location)
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
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._interface.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

# The definition of the recharging state.
# It is the state where the robot recharges its battery
class Recharging(State):

    # Construct this class, i.e., initialise this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interfaces with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_BATTERY_LOW, TRANS_CHARGED_BATTERY, 
                                       TRANS_SEARCH_CHARG_LOC], input_keys=[VAR_CHARG_FEEDBACK])

    

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        
        # check if the robot is already in the charging location
        robot_is = self._interface.ask_rob_pos()
        if robot_is == self._interface.charging_loc:
            self._interface.found = True
            self._interface.set_starting_flag(self._interface.found)
            print(Fore.WHITE)
            print(Fore.LIGHTGREEN_EX + "The Robot is in the Charging Location.")
            print()
            print(Fore.LIGHTGREEN_EX + "...Wait for charging...")
            print(Fore.WHITE)            
        else:
            print(Fore.WHITE)
            print(Fore.LIGHTCYAN_EX + "Searching the charging location mode...")
            print(Fore.WHITE)
            self._interface.low_battery_mode = True

         
        

        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._interface.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._interface.is_battery_low():
                    print()
                    print((Fore.LIGHTCYAN_EX + "The battery is charged."))
                    print()
                    self._interface.reset_states()  # Reset the state variable related to the stimulus.
                    self._interface.set_starting_flag(self._interface.found)
                    return TRANS_CHARGED_BATTERY
                if self._interface.is_battery_low() and not self._interface.found:
                    return TRANS_SEARCH_CHARG_LOC
                #if self._interface.is_battery_low() and feedback:
                #    return TRANS_BATTERY_LOW
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except `TRAMS_RECHARGED`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._interface.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the wait state.
# It is the state that simulate that the robot are doing something in the goal room
class Wait(State):

    # Construct this class, i.e., initialise this state.
    def __init__(self,state_machine_interface):

        # Get a reference to the interfaces with the other nodes of the architecture.
        self._interface = state_machine_interface

        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRANS_TIMEOUT, TRANS_BATTERY_LOW, TRANS_IN_A_ROOM])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        waiting_time = rospy.get_param("/waiting_time")
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._interface.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if self._interface.is_battery_low():
                    return TRANS_BATTERY_LOW
                else:
                    print(Fore.WHITE)
                    print(Fore.CYAN + "...WAITING...")
                    print(Fore.WHITE)
                    rospy.sleep(waiting_time)
                    return TRANS_TIMEOUT
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except `TRAMS_RECHARGED`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._interface.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


def main():

    # Initialise this ROS node.
    rospy.init_node('state_machine_russo_gabriele')

    # Initialise an interface class to manage the interfaces with the other nodes in the architectures, i.e., it manages external stimulus.
    interface = Handler()

    # Define the structure of the Finite State Machine.
    sm = StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    with sm:

        # Define the inner state to plan the via-points toward a random position.
        StateMachine.add(STATE_WAIT_FOR_TOP_MAP, WaitForTopMap(interface),
                            transitions={TRANS_NO_TOP_MAP: STATE_WAIT_FOR_TOP_MAP,
                                        TRANS_TOP_MAP_READY: STATE_PLANNING})

        # Define the inner state to move to a random position.
        StateMachine.add(STATE_PLANNING, Planning(interface),
                            transitions={TRANS_TOP_MAP_READY: STATE_PLANNING,
                                        TRANS_MOVING: STATE_MOVING,
                                        TRANS_CHARGED_BATTERY: STATE_PLANNING,
                                        TRANS_BATTERY_LOW: STATE_RECHARGING,
                                        TRANS_SEARCH_CHARG_LOC: STATE_PLANNING,
                                        TRANS_TIMEOUT: STATE_PLANNING})
        
        # Define the inner state to move to a random position.
        StateMachine.add(STATE_MOVING, Moving(interface),
                            transitions={TRANS_BATTERY_LOW: STATE_RECHARGING,
                                        TRANS_MOVING: STATE_MOVING,
                                        TRANS_IN_A_ROOM: STATE_WAIT})

        # Define the inner state to plan the via-points toward the user.
        StateMachine.add(STATE_WAIT, Wait(interface),
                            transitions={TRANS_TIMEOUT: STATE_PLANNING,
                                        TRANS_BATTERY_LOW: STATE_RECHARGING,
                                        TRANS_IN_A_ROOM: STATE_WAIT})

        # Define the inner state to move toward the user.
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