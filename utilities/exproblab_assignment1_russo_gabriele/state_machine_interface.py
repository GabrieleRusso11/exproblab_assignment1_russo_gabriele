#!/usr/bin/env python

"""
.. module:: state_machine_interface       
   :platform: Ubuntu 20.04
   :synopsis: Python module which implement an interaface used to handle the comunication between the state machine node and the other components of the architecture

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

This module implements an interaface used to handle the comunication between 
the state machine node and the other components of the architecture but it also
contains functions used by the state machine node to compute data and to interact 
with the ontology through the Armor server

ROS Parameters:
  **/starting_map** starting map file (it has the Ontology skeleton ) \n
  **/operative_map** is the file where the final map (and the complete ontology) will be loaded \n
  **/location_property** property "hasDoor" associated to a location (room and corridor) \n
  **/robot_property** property "isIn" to get or set the robot position in the map \n
  **/now_property** property "now" is a robot time property \n
  **/urgent_property** property "urgencyThreshold" to change the urgency threshold, associated to the locations \n
  **/visited_property** property "visitedAt" used to change the timestamp of a location, when the robot visits it \n
  **/canreach_property** property "canReach" to get the locations that the robot can reach \n
  **/robot_name** the name that identify the robot \n
  **/charging_location** the name of the location where is the charging station \n

Subscriber:
  **/state/battery_low** receives from the robot state node the robot battery level \n

Service:
  **/inteface/start_charging** sends to the robot state node, the value of the start charging flag, as service request, in order to handle the robot charging process .\n

Action Client:
  **/motion/controller** sends to the controller node the path plan as goal, and receives the final location reached as result. \n
  **/motion/planner** sends to the planner node the new location to reach as goal, and receives the path plan to reach the new location as result. \n

"""

# Import libraries.
import sys
import os
import random
import re
import rospy
import time
import datetime
import calendar

# Import the simple ActionClient.
from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# Import constant names that define the architecture's structure.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import ROS-based messages.
from std_msgs.msg import Bool
from std_msgs.msg import String

# Import custom message, actions and services.
from exproblab_assignment1_russo_gabriele.msg import PlanAction, ControlAction, Location
from exproblab_assignment1_russo_gabriele.srv import NewPosition, StartCharging

from armor_api.armor_client import ArmorClient
from armor_msgs.msg import ArmorDirectiveReq

from os.path import dirname, realpath

import numpy as np

# to color the text
import colorama
from colorama import Fore

# get the starting map from the parameter server
s_map = rospy.get_param("/starting_map")

# get the file where the user map will be loaded
# from the parameter server
op_map = rospy.get_param("/operative_map")

# Definition of the path used to load the map file in 
# the armor server for the map creation 
path = dirname(realpath(__file__))
path = os.path.abspath(os.path.join(path, os.pardir))
path = path + "/../topological_map/"


# A tag for identifying logs producer.
LOG_TAG = 'state-machine' + '-HELPER'



class ActionClientHelper:
    """
    A class to simplify the implementation of a 
    simple action client for ROS action servers.
    """

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        """
        Class constructor

        Args:
          service_name : it is the name of the server that will be invoked by this client.
          action_type : it is the message type that the server will exchange.
          done_callback : it is the name of the function called when the action server completed its computation. If this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be called when the server completes its computation.
          feedback_callback : it is the name of the function called when the action server sends a feedback message. If this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be called when the server sends a feedback message.
          mutex : it is a 'Lock' object synchronised with the `done_callback` and `feedback_callback`. If it is not set (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the synchronization with other classes.

        Note: This class was implemented by the professor Luca Buoncompagni (https://github.com/buoncubi/arch_skeleton)  
        """
        # Initialize the state of this client: `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()

        # Set the name of the server to be invoked.
        self._service_name = service_name

        # Get or create a new mutex.
        if mutex is None:

            self._mutex = Lock()

        else:

            self._mutex = mutex
        
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)

        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback

        # Wait for the action server to be alive.
        self._client.wait_for_server()

    
    def send_goal(self, goal):
        """
        Start the action server with a new `goal`. 
        Note: this call is not blocking (i.e., asynchronous performed).
        """
        # A new goal can be given to the action server only if it is not running.
        # This simplification implies that within the ROS architecture no more 
        # than one client can use the same server at the same time.
        if not self._is_running:

            # Start the action server (sending the goal).
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None

        else:

            warn_msg = 'Warning send a new goal, cancel the current request first!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    def cancel_goals(self):
        """
        Stop the computation of the action server (cancels the goal sent).
        """

        # The computation can be stopped only if the server is actually computing.
        if self._is_running:

            # Stop the computation.
            self._client.cancel_all_goals()

            # Reset the client's state.
            self.reset_client_states()

        else:

            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    def reset_client_states(self):
        """
        Reset the client state variables stored in this class.
        """
        self._is_running = False
        self._is_done = False
        self._results = None

    def _feedback_callback(self, feedback):
        """
        This function is called when the action server send some `feedback` to the client.
        """

        # Acquire the mutex to synchronise the computation concerning the `feedback` 
        # message with the other nodes of the architecture.
        self._mutex.acquire()

        try:

            # Eventually, call the method provided by the node that uses this 
            # action client to manage a feedback.
            if self._external_feedback_cb is not None:

                self._external_feedback_cb(feedback)
            
        finally:

            # Realise the mutex to (eventually) unblock ROS-based 
            # thread waiting on the same mutex.
            self._mutex.release()

    def _done_callback(self, status, results):
        """
        This function is called when the action server finish its computation,
        i.e., it provides a `done` message.
        """

        # Acquire the mutex to synchronise the computation concerning 
        # the `done` message with the other nodes of the architecture.
        self._mutex.acquire()

        try:

            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results

            # Eventually, call the method provided by the node that 
            # uses this action client to manage a result.
            if self._external_done_cb is not None:

                self._external_done_cb(status, results)
            
        finally:

            self._mutex.release()

    # These three function should be mutex safe
    def is_done(self):  
        """
        Get `True` if the action server finished is computation, or `False` otherwise.
        """
        return self._is_done

    def is_running(self):
        """
        Get `True` if the action server is running, or `False` otherwise.
        """
        return self._is_running

    def get_results(self):
        """
        Get the results of the action server, if any, or `None`.
        """
        if self._is_done:

            return self._results
        
        else:

            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, LOG_TAG))

            return None

class Handler:
    """
    The class Handler is used as an interface of the state machine to handle 
    the comunication with the other nodes but also to contains function used by
    the state machine to compute data, for instance the creation of the ontology
    and the retrieving of information from the ontology itself.
    """
    
    def __init__(self):
        """
        Class constructor, i.e., class initializer.
        It creates a shared mutex to synchronize action clients and subscribers.
        Then it sets the initial state involving the 'self._battery_low','self.found' 
        and 'self.low_battery_mode' variables, gets some parameters from the Ros server parameter
        and initialize the main variables odìf the class.
        Lastly it defines the subscriber to the robot state node topic and the two Action client 
        motion/controller and motion/planning
        """

        # Create a shared mutex to synchronize action clients and subscribers.
        # Note that, based on different assumptions, further optimization 
        # can be done to make the different threads blocking for a less 
        # amount of time in the same mutex.
        self.mutex = Lock()

        # Set the initial state involving the `self._battery_low`, 
        # `self.found` and `self.low_battery_mode` variables.
        self.reset_states()

        # object properties
        self.loc_prop = rospy.get_param("/location_property")
        self.rob_prop = rospy.get_param("/robot_property")
        self.now_prop = rospy.get_param("/now_property")

        # data properties
        self.urg_th = rospy.get_param("/urgent_property")
        self.vis = rospy.get_param("/visited_property")
        self.connection = rospy.get_param("/canreach_property")

        self.rob = rospy.get_param("/robot_name")
        self.type_value = "LONG"
        self.old_value = "7"

        # initialize the initial robot position
        self._init_pos = ''

        # will contain all the corridors and rooms 
        self.loc = []

        # number of corridors and rooms
        self.num_corr = 0
        self.num_room = 0

        # initialization of the armor client
        self._client = self.init_client()

        # initialization of the robot charging location
        self.charging_loc = rospy.get_param("/charging_location")

        # Define the callback associated with the battery low ROS subscriber.
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)

        # Define the clients for the plan and control action servers.
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    def reset_states(self):
        """
        Reset the states variable (_battery_low, found and low_battery_mode).
        """
        self._battery_low = False
        self.found = False
        self.low_battery_mode = False
        
    def _battery_callback(self, msg):
        """
        The subscriber to get messages published from the `robot-state` node 
        into the `/state/battery_low/` topic.

        Args:
          msg (bool) : the battery level value

        """

        # Acquire the mutex to assure the synchronization with the other 
        # subscribers and action clients (this assure data consistency).
        self.mutex.acquire()

        try:

            # Get the battery level  
            self._battery_low = msg.data
        
        finally:

            # Release the mutex to eventually unblock the other 
            # subscribers or action servers that are waiting.
            self.mutex.release()

    def is_battery_low(self):
        """
        Get the state variable encoded in this class that concerns the battery level.

        Returns:
          self._battery_low (bool) : battery level. 'True' if the battery is low, 'False' otherwise.
        """
        return self._battery_low

    def set_starting_flag(self,feedback):
        """
        Update the start charging flag stored in the 'robot-state' node.
        This method is used to say if the robot state can change the battery level
        in order to start the charging battery simulation only when the robot has
        reached the charging location.

        Args:
          feedback (bool) : the value of the start charging flag. If it is 'True' the robot can start the charging process, 'False' the robot has to wait.
        """

        # wait for the server to be initialized.
        rospy.wait_for_service(anm.SERVER_CHARGING)

        try:

            # Call the service and set the start charging flag.
            service = rospy.ServiceProxy(anm.SERVER_CHARGING, StartCharging)
            service(feedback)  # The response is not used.

        except rospy.ServiceException as e:

            log_msg = f'Server cannot set the starting flag: {e}'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

# ******************************** SECTION WITH THE METHODS USED TO CREATE THE MAP ********************************

    # This method initialize the armor client, 
    # used to load and to communicate with the ontology
    def init_client(self):
        """
        This method initialize the armor client, 
        used to load and to communicate with the ontology

        Returns:
          client : Armor client identifier
        """

        client = ArmorClient("create_map", "top_map") # "create_map" is the client id (name) 
                                                      # and "top_map" is the reference name that will 
                                                      # be used to always refer to the same ontology 
                                                      # loaded on memory.

        # load the .owl file to create the map 
        client.utils.load_ref_from_file(path + s_map, "http://bnc/exp-rob-lab/2022-23", buffered_manipulation=True, reasoner='PELLET', buffered_reasoner=True, mounted=False)

        client.utils.mount_on_ref()
        client.utils.set_log_to_terminal(True)

        return client

    def add_corridors(self):
        """
        This method adds locations in the CORRIDOR subclass,
        so it adds corridors in the ontology.
        Note: a location to be a corridor must have at least two doors
        """
        i = 0
        j = 0

        # loop in order to insert the corridors, stated by the user
        for i in range(self.num_corr):
            
            print(Fore.WHITE)
            print(Fore.LIGHTGREEN_EX + "Insert the ", Fore.LIGHTRED_EX + "corridor", Fore.LIGHTGREEN_EX + " : ")
            self.loc.append(input(Fore.WHITE))

            print(Fore.WHITE)
            print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTYELLOW_EX + "doors", Fore.LIGHTGREEN_EX + "have the" ,Fore.LIGHTRED_EX + f"corridor {self.loc[i]}", Fore.LIGHTGREEN_EX + "? ")

            dim_dc = int(input(Fore.WHITE))

            # loop in order to insert the doors of each corridor
            for j in range(dim_dc):

                print(Fore.WHITE)

                print(Fore.LIGHTGREEN_EX + "Insert the ", Fore.LIGHTYELLOW_EX + "door", Fore.LIGHTGREEN_EX + " : ")
                do = input(Fore.WHITE)

                self._client.manipulation.add_objectprop_to_ind(self.loc_prop, self.loc[i], do)

    def add_rooms(self):
        """
        This method adds locations in the ROOM subclass,
        so it adds rooms in the ontology.
        Note: a location to be a room must have maximum one door
        """
        
        i = 0
        j = 0

        print(Fore.WHITE)

        # loop in order to insert the rooms, stated by the user,
        # and the doors for each room
        for i in range(self.num_room):

            print(Fore.LIGHTGREEN_EX + "Insert the", Fore.LIGHTRED_EX + " room", Fore.LIGHTGREEN_EX + " : ")
            self.loc.append(input(Fore.WHITE))
            print()
 
            print(Fore.LIGHTGREEN_EX + "Insert the ",Fore.LIGHTYELLOW_EX + "door", Fore.LIGHTGREEN_EX + "of the " ,Fore.LIGHTRED_EX + f"room {self.loc[self.num_corr + i]}", Fore.LIGHTGREEN_EX + " : ")
            do = input((Fore.WHITE))

            self._client.manipulation.add_objectprop_to_ind(self.loc_prop, self.loc[self.num_corr + i], do)

            print()

    def disjoint(self):
        """
        This method disjoint all the individuals
        """

        self._client.manipulation.disj_inds_of_class('LOCATION')
        self._client.manipulation.disj_inds_of_class('ROOM')
        self._client.manipulation.disj_inds_of_class('DOOR')
        self._client.manipulation.disj_inds_of_class('CORRIDOR')
        self._client.manipulation.disj_inds_of_class('URGENT')
    
    def map_creation(self):
        """
        This method uses the 'add_corridors' and 'add_rooms' 
        methods to allow the user to create the map
        """

        print()
        print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTRED_EX + "corridors", Fore.LIGHTGREEN_EX + "are there? ")

        self.num_corr = int(input(Fore.WHITE))

        self.add_corridors()

        print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTRED_EX + "rooms", Fore.LIGHTGREEN_EX + "are there? ")

        self.num_room = int(input(Fore.WHITE))
        
        self.add_rooms()

        self.disjoint()

    def change_urg_th(self):
        """
        This method allows the user to change the location urgency threshold.
        """
        
        print(Fore.LIGHTGREEN_EX + "Insert the",Fore.LIGHTYELLOW_EX + " desired threshold",Fore.LIGHTGREEN_EX + " (it must be greater than 15): ")
        th_new = input(Fore.WHITE)

        print(Fore.WHITE)

        self._client.manipulation.remove_dataprop_from_ind(self.urg_th, self.rob, self.type_value, self.old_value)
        self._client.manipulation.add_dataprop_to_ind(self.urg_th, self.rob, self.type_value, th_new)

    def loc_visit_time(self):
        """
        This method initializes the locations visited time
        """

        # take the datetime to initialize each visited time of the 
        # locations through visitedAt
        date = datetime.datetime.utcnow()
        utc_time = calendar.timegm(date.utctimetuple())
        new_time = str(utc_time)

        # initiliaze the timestamp of each location (corridors and rooms)
        # thanks to the data property visitedAt
        i = 0
        for i in range(len(self.loc)):

            self._client.manipulation.add_dataprop_to_ind(self.vis, self.loc[i], self.type_value, new_time)

    def init_rob_pos(self):
        """
        This method allows the user to choose the initial position 
        among the inserted locations.
        For this assignment the initial position is the corridor E
        """

        print(Fore.LIGHTYELLOW_EX + "This is the list of all",Fore.LIGHTRED_EX + "locations", Fore.LIGHTYELLOW_EX + " (first corridors, then rooms) : ",Fore.WHITE)
        print(*self.loc)
        print(Fore.WHITE)

        print(Fore.LIGHTYELLOW_EX + "There are ", Fore.LIGHTRED_EX + f"{len(self.loc)} locations ")
        print()
        print(Fore.LIGHTYELLOW_EX + "Insert the number of the ", Fore.LIGHTRED_EX + "initial location", Fore.LIGHTYELLOW_EX + " where robot will start to move", Fore.LIGHTGREEN_EX + f" (1 for the first location of the list until {len(self.loc)} that is the last location of the list) : ")
        choice = int(input(Fore.WHITE))

        print()

        self._init_pos = self.loc[choice - 1]

        self._client.manipulation.add_objectprop_to_ind(self.rob_prop, self.rob, self.loc[choice - 1])

    def reasoner(self):
        """
        Activate the reasoner 
        """

        print()
        self._client.utils.sync_buffered_reasoner()
        self._client.utils.apply_buffered_changes()
        print()

    def set_map(self):
        """
        This method create the map using the 'map_creation()' method 
        and set all the other main parameters through the previous 
        methods of this section 
        """

        # the user inserts the data to create the desired map
        self.map_creation()

        # possible modification of the location urgency threshold 
        print(Fore.LIGHTGREEN_EX + "Do you want to change", Fore.LIGHTYELLOW_EX + "the urgency threshold", Fore.LIGHTGREEN_EX + "(y or n) ? ")
        res = input(Fore.WHITE)
        print(Fore.WHITE)

        if res == "y" or res == "Y" :

            # change the location urgency threshold 
            self.change_urg_th()

        else :

            print("NO changes")
            print()

            pass

        # initialize the timestamp of each location (corridors and rooms) thanks to the data property visitedAt
        self.loc_visit_time()

        # initialize the initial position of the robot
        self.init_rob_pos()
        
        self.reasoner()

        self.end = False

        while(not self.end):

            print()
            print(Fore.LIGHTYELLOW_EX + "Is the map ready? :")
            c = input(Fore.WHITE)
            print(Fore.WHITE)

            if c == 'y' or c == 'Y':

                # save the map created by the user
                self._client.utils.save_ref_with_inferences(path + op_map)
                self.end = True

            else :

                print(Fore.LIGHTYELLOW_EX + "The map is not ready or you have written the wrong character")
                print(Fore.LIGHTYELLOW_EX + "Please if the map is ready, write y or Y")

# ******************************* SECTION WITH THE METHODS USED TO OPERATE ON THE MAP *******************************

    def can_reach(self):
        """
        This method returns the locations connected 
        to the location where the robot is, then
        the locations that the robot can reach.

        Returns:
          reachable (string) : Locations that the robot can reach
        """
        reachable = self._client.query.objectprop_b2_ind(self.connection, self.rob)

        return reachable
    
    def urgent(self):
        """
        This method returns the urgent locations to visit

        Returns:
          urgent (string) : Urgent locations to visit
        """
        urgent = self._client.query.ind_b2_class("URGENT")

        return urgent
    
    def corridor(self):
        """
        This method returns the list of corridors 

        Returns:
          corridor (string) : list of corridors
        """
        corridor = self._client.query.ind_b2_class("CORRIDOR")

        return corridor
                
    def get_time(self):
        """
        This method gets the current time 
        in order to update the timestamps

        Returns:
          now : the current time
        """
        now = str(int(time.time()))

        return now
    
    def update_robot_time(self):
        """
        This method updates the robot time
        """

        old_time_now = self._client.query.dataprop_b2_ind(self.now_prop, self.rob)[0]
        old_time_now = re.findall(r'"(.+?)"', old_time_now)[0]
        new_time = self.get_time()
        
        self._client.manipulation.remove_dataprop_from_ind(self.now_prop,self.rob,self.type_value, old_time_now)
        self._client.manipulation.add_dataprop_to_ind(self.now_prop, self.rob, self.type_value, new_time)

    def update_location_time(self, new_loc):
        """
        This method updates the location time
        """

        old_time_vis = self._client.query.dataprop_b2_ind(self.vis, new_loc)[0]
        old_time_vis = re.findall(r'"(.+?)"', old_time_vis)[0]
        new_time = self.get_time()
        
        self._client.manipulation.remove_dataprop_from_ind(self.vis, new_loc, self.type_value, old_time_vis)
        self._client.manipulation.add_dataprop_to_ind(self.vis, new_loc, self.type_value, new_time)

    def update_robot_position(self, old_loc, new_loc):
        """
        This method updates the robot position
        """

        self._client.manipulation.remove_objectprop_from_ind(self.rob_prop, self.rob, old_loc)
        self._client.manipulation.add_objectprop_to_ind(self.rob_prop, self.rob, new_loc)

    def ask_rob_pos(self):
        """
        This method returns the current robot position

        Returns:
          rob_pos (string) : current robot position
        """

        rob_pos = self._client.query.objectprop_b2_ind(self.rob_prop, self.rob)[0]

        return rob_pos
    
    def go_in_a_corridor(self,corr,can_reach,goal):
        """
        This method seach a corridor among the reachable locations
        """
        i = 0
        j = 0
        for i in range(len(can_reach)):

            for j in range(len(corr)):

                if can_reach[i] == corr[j]:

                    goal.append(can_reach[i])

        if goal == [] :

            goal.append(random.choice(can_reach))

    def target_location(self):
        """
        This method returns the target location, 
        i.e. the location that the robot should reach
        considering the corridors, the urgent locations and
        obviously the reachable locations

        Returns:
          plan (Location) : the next location to reach

        """
        i=0
        j=0

        reachable_urg_loc = []
        goal_loc = []

        rospy.sleep(1)

        # locations that the robot can reach 
        reachable_loc = self.can_reach()

        # urgent locations
        urgent_loc = self.urgent()

        # corridors
        corridors = self.corridor()

        print(Fore.LIGHTYELLOW_EX + "the robot can reach the following locations : ", Fore.WHITE)
        print(*reachable_loc)
        print()

        print(Fore.LIGHTRED_EX + "corridors", Fore.LIGHTYELLOW_EX + " are : ", Fore.WHITE)
        print(*corridors)
        print()

        # check if there are urgent locations
        if len(urgent_loc) > 0 :

            print(Fore.LIGHTCYAN_EX + "urgent locations", Fore.LIGHTYELLOW_EX + " are : ", Fore.WHITE)
            print(*urgent_loc)
            print()

            # check if the urgent locations are reachable 
            for i in range(len(reachable_loc)):

                for j in range(len(urgent_loc)):

                    if reachable_loc[i] == urgent_loc[j]:
                        reachable_urg_loc.append(reachable_loc[i])
            
            # urgent locations reachable
            if len(reachable_urg_loc) > 0 :

                print(Fore.LIGHTCYAN_EX + "urgent location", Fore.LIGHTYELLOW_EX + " found ", Fore.WHITE)
                print()
                print(Fore.LIGHTYELLOW_EX + "Go in the ",Fore.LIGHTCYAN_EX + "urgent location")
                print(Fore.WHITE)

                self.go_in_a_corridor(corridors,reachable_urg_loc,goal_loc)

                print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ")
                
                # The target location is a random choice between the urgent locations
                plan = random.choice(goal_loc)
                print(plan)

            else : # if no urgent location is reachable, then go in a corridor

                print(Fore.LIGHTYELLOW_EX + "No ",Fore.LIGHTCYAN_EX + "urgent location", Fore.LIGHTYELLOW_EX + "reachable")
                print()
                print(Fore.LIGHTYELLOW_EX + "Go in a ",Fore.LIGHTRED_EX + "reachable corridor ")
                print()
                
                self.go_in_a_corridor(corridors,reachable_loc,goal_loc)

                print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ")

                plan = random.choice(goal_loc)
                print(plan)

        else : # if there are no urgent locations, then go in a corridor

            print(Fore.LIGHTYELLOW_EX + "there are ",Fore.LIGHTCYAN_EX + "no urgent location",Fore.LIGHTYELLOW_EX + " at moment")
            print()
            print(Fore.LIGHTYELLOW_EX + "Go in a ",Fore.LIGHTRED_EX + "reachable corridor ")
            print()

            self.go_in_a_corridor(corridors,reachable_loc,goal_loc)

            print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ",Fore.WHITE)

            plan = random.choice(goal_loc)
            print(plan)
        return Location(plan)
    
    def search_charging_location(self):
        """
        This method is used to search the charging location,
        when the robot is in the low battery mode.
        Therefore the robot has to go in the charging location.

        Returns:
          target (Location) : the next location to reach when the robot has a low battery level
        """

        i = 0
        target = ''

        print()
        print(Fore.LIGHTCYAN_EX + "Searching the charging location...")
        print()

        # location that the robot can reach 
        reachable_loc = self.can_reach()

        # corridors
        corridors = self.corridor()
        
        print(Fore.LIGHTYELLOW_EX + "the robot can reach the following locations : ", Fore.WHITE)
        print(*reachable_loc)
        print()

        print(Fore.LIGHTRED_EX + "corridors", Fore.LIGHTYELLOW_EX + " are : ", Fore.WHITE)
        print(*corridors)
        print()
        
        # check if among the reachable locations, 
        # there is the charging location
        for i in range(len(reachable_loc)):

            if reachable_loc[i] == self.charging_loc[0]:

                target = reachable_loc[i]

                print()
                print(Fore.LIGHTCYAN_EX + "Charging Location found !!!")
                print()

                print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ",Fore.WHITE)
                print(target)

                return Location(target)
        
        i = 0
        j = 0
        reachable_corridors = []

        # if the charging location is not reachable,
        # then go in a reachable corridor and continue
        # to search the charging location
        if len(target) == 0:

            print(Fore.LIGHTYELLOW_EX + "Charging Location not found", Fore.WHITE)

            for i in range(len(reachable_loc)):

                for j in range(len(corridors)):

                    if reachable_loc[i] == corridors[j]:

                        reachable_corridors.append(reachable_loc[i])

            # if no corridor is reachable, go in a random reachable location
            if len(reachable_corridors) == 0:

                print(Fore.LIGHTYELLOW_EX + "Go in a random reachable location", Fore.WHITE)
                print()

                target = random.choice(reachable_loc)

                print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ",Fore.WHITE)
                print(target)

                return Location(target)
            
            else:

                print(Fore.LIGHTYELLOW_EX + "Go in a corridor", Fore.WHITE)
                print()

                target= random.choice(reachable_corridors)

                print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ",Fore.WHITE)
                print(target)

                return Location(target)

        