#!/usr/bin/env python

import sys
import os
import random
import re

# Import ROS libraries.
import rospy
import time
import datetime
import calendar

from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# Import constant names that define the architecture's structure.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import ROS-based messages.
from std_msgs.msg import Bool
from std_msgs.msg import String

from exproblab_assignment1_russo_gabriele.msg import PlanAction, ControlAction, Location
from exproblab_assignment1_russo_gabriele.srv import NewPosition, StartCharging

from armor_api.armor_client import ArmorClient
from armor_msgs.msg import ArmorDirectiveReq

from os.path import dirname, realpath

import numpy as np

# to color the text
import colorama
from colorama import Fore

s_map = rospy.get_param("/starting_map")
op_map = rospy.get_param("/operative_map")

# Initialize reference
path = dirname(realpath(__file__))
path = os.path.abspath(os.path.join(path, os.pardir))
path = path + "/../topological_map/"


# A tag for identifying logs producer.
LOG_TAG = 'state-machine' + '-HELPER'


# A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
class ActionClientHelper:
    # Class constructor, i.e., class initializer. Input parameters are:
    #  - `service_name`: it is the name of the server that will be invoked by this client.
    #  - `action_type`: it is the message type that the server will exchange.
    #  - `done_callback`: it is the name of the function called when the action server completed its computation. If
    #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
    #     called when the server completes its computation.
    #  - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
    #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
    #    called when the server sends a feedback message.
    #  - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
    #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
    #    synchronization with other classes.
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
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

    # Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
    def send_goal(self, goal):
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
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

    # Stop the computation of the action server.
    def cancel_goals(self):
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    # Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server send some `feedback` back to the client.
    def _feedback_callback(self, feedback):
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    # This function is called when the action server finish its computation, i.e., it provides a `done` message.
    def _done_callback(self, status, results):
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
            # Uncomment below to log information.
            # log_msg = f'`{self._service_name}` done with state `{self._client.get_state_txt()}` and result: {results}.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            self._mutex.release()

    # Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    # Get `True` if the action server is running, or `False` otherwise.
    # A note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    # Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, LOG_TAG))
            return None


# A class to decouple the implementation of the Finite State Machine to the stimulus might that
# lead to state transitions. This class manages the synchronization with subscribers and action
# servers.
class Handler:
    # Class constructor, i.e., class initializer.
    def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers.
        # Note that, based on different assumptions, further optimization can be done to make the different threads
        # blocking for a less amount of time in the same mutex.
        self.mutex = Lock()
        # Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
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

        self._init_pos = ''
        # will contain all the corridors and rooms 
        self.loc = []

        # number of corridors and rooms
        self.num_corr = 0
        self.num_room = 0

        self._client = self.init_client()
        self.charging_loc = rospy.get_param("/charging_location")

        # Define the callback associated with the battery low ROS subscriber.
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)

        # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    # Reset the stimulus, which are stored as states variable fo this class.
    # This function assumes that no states of the Finite State Machine run concurrently.
    def reset_states(self):
        self._battery_low = False
        self.found = False
        self.low_battery_mode = False
        

    # The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
    def _battery_callback(self, msg):
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
            # Uncomment below to log data.
            #if self._battery_low:
                #log_msg = 'Robot with low battery.'
            #else:
                #log_msg = 'Robot battery fully charged.'
            #rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    # Get the state variable encoded in this class that concerns the battery level.
    # The returning value will be `True` if the battery is low, `False` otherwise.
    # Note that the node using this class might exploit the `reset_state` function to improve robustness.
    # Also note that this function should be used when the `mutex` has been acquired. This assures the
    # synchronization  with the threads involving the subscribers and action clients.
    def is_battery_low(self):
        return self._battery_low

    def set_starting_flag(self,feedback):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_CHARGING)
        try:
            service = rospy.ServiceProxy(anm.SERVER_CHARGING, StartCharging)
            service(feedback)  # The `response` is not used.
        except rospy.ServiceException as e:
            log_msg = f'Server cannot set the starting flag: {e}'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

    # methods to create the map
    def init_client(self):

        client = ArmorClient("create_map", "top_map") # "create_map" is the client id (name) 
                                                    # and "top_map" is the reference name hat will 
                                                    # be used to always refer to the same ontology 
                                                    # loaded on memory.

        # load the .owl file to create the map 
        client.utils.load_ref_from_file(path + s_map, "http://bnc/exp-rob-lab/2022-23", buffered_manipulation=True, reasoner='PELLET', buffered_reasoner=True, mounted=False)

        client.utils.mount_on_ref()
        client.utils.set_log_to_terminal(True)

        return client

    def add_corridors(self):

        # adding corridors to the map (in the CORRIDOR subclass)
        # adding doors to corridors
        i = 0
        j = 0
        for i in range(self.num_corr):

            print(Fore.WHITE)
            print(Fore.LIGHTGREEN_EX + "Insert the ", Fore.LIGHTRED_EX + "corridor", Fore.LIGHTGREEN_EX + " : ")
            self.loc.append(input(Fore.WHITE))

            print(Fore.WHITE)
            print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTYELLOW_EX + "doors", Fore.LIGHTGREEN_EX + "have the" ,Fore.LIGHTRED_EX + f"corridor {self.loc[i]}", Fore.LIGHTGREEN_EX + "? ")

            dim_dc = int(input(Fore.WHITE))

            for j in range(dim_dc):

                print(Fore.WHITE)

                print(Fore.LIGHTGREEN_EX + "Insert the ", Fore.LIGHTYELLOW_EX + "door", Fore.LIGHTGREEN_EX + " : ")
                do = input(Fore.WHITE)

                self._client.manipulation.add_objectprop_to_ind(self.loc_prop, self.loc[i], do)

    def add_rooms(self):
        # adding rooms to the map (in the ROOM subclass)
        # adding doors to rooms
        i = 0
        j = 0
        print(Fore.WHITE)
        for i in range(self.num_room):

            print(Fore.LIGHTGREEN_EX + "Insert the", Fore.LIGHTRED_EX + " room", Fore.LIGHTGREEN_EX + " : ")
            self.loc.append(input(Fore.WHITE))
            print()
            print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTYELLOW_EX + "doors", Fore.LIGHTGREEN_EX + "have the" ,Fore.LIGHTRED_EX + f"room {self.loc[self.num_corr + i]}", Fore.LIGHTGREEN_EX + "? ")

            dim_dr = int(input(Fore.WHITE))

            for j in range(dim_dr):

                print(Fore.WHITE)
                print(Fore.LIGHTGREEN_EX + "Insert the ",Fore.LIGHTYELLOW_EX + "door", Fore.LIGHTGREEN_EX + " : ")
                do = input((Fore.WHITE))

                self._client.manipulation.add_objectprop_to_ind(self.loc_prop, self.loc[self.num_corr + i], do)
            print()

    def disjoint(self):
        self._client.manipulation.disj_inds_of_class('LOCATION')
        self._client.manipulation.disj_inds_of_class('ROOM')
        self._client.manipulation.disj_inds_of_class('DOOR')
        self._client.manipulation.disj_inds_of_class('CORRIDOR')
        self._client.manipulation.disj_inds_of_class('URGENT')

    def map_creation(self):
        print()
        print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTRED_EX + "corridors", Fore.LIGHTGREEN_EX + "are there? ")

        self.num_corr = int(input(Fore.WHITE))

        self.add_corridors()

        print(Fore.LIGHTGREEN_EX + "How many", Fore.LIGHTRED_EX + "rooms", Fore.LIGHTGREEN_EX + "are there? ")

        self.num_room = int(input(Fore.WHITE))
        
        self.add_rooms()

        self.disjoint()

    def change_urg_th(self):
        
        print(Fore.LIGHTGREEN_EX + "Insert the",Fore.LIGHTYELLOW_EX + " desired threshold",Fore.LIGHTGREEN_EX + " (it must be greater than 15): ")
        th_new = input(Fore.WHITE)

        print(Fore.WHITE)

        self._client.manipulation.remove_dataprop_from_ind(self.urg_th, self.rob, self.type_value, self.old_value)
        self._client.manipulation.add_dataprop_to_ind(self.urg_th, self.rob, self.type_value, th_new)


    def loc_visit_time(self):

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

        print(Fore.LIGHTYELLOW_EX + "This is the list of all",Fore.LIGHTRED_EX + "locations", Fore.LIGHTYELLOW_EX + " (first corridors, then rooms) : ",Fore.WHITE)
        print(*self.loc)
        print(Fore.WHITE)
        print(Fore.LIGHTYELLOW_EX + "There are ", Fore.LIGHTRED_EX + f"{len(self.loc)} locations ")
        print()
        print(Fore.LIGHTYELLOW_EX + "Insert the number of the ", Fore.LIGHTRED_EX + "initial location", Fore.LIGHTYELLOW_EX + " where robot will start to move", Fore.LIGHTGREEN_EX + f" (1 for the first location of the list until {len(self.loc)} that is the last location of the list) : ")
        choice = int(input(Fore.WHITE))

        print()

        self._init_pos = self.loc[choice - 1]

        # for this assignment the initial position is the corridor E
        self._client.manipulation.add_objectprop_to_ind(self.rob_prop, self.rob, self.loc[choice - 1])

    def set_map(self):

        self.map_creation()

        # possible change of the urgency threshold of the room
        print(Fore.LIGHTGREEN_EX + "Do you want to change", Fore.LIGHTYELLOW_EX + "the urgency threshold", Fore.LIGHTGREEN_EX + "(y or n) ? ")
            
        res = input(Fore.WHITE)

        print(Fore.WHITE)

        if res == "y" or res == "Y" :

            self.change_urg_th()

        else :
            print("NO changes")
            print()
            pass

        # initialize the timestamp of each location (corridors and rooms) thanks to the data property visitedAt
        self.loc_visit_time()

        # initialize the initial position of the robot
        self.init_rob_pos()
        
        self._client.utils.apply_buffered_changes()
        self._client.utils.sync_buffered_reasoner()

        self.end = False
        while(not self.end):
            print()
            print(Fore.LIGHTYELLOW_EX + "Is the map ready? :")
            c = input(Fore.WHITE)
            print(Fore.WHITE)
            if c == 'y' or c == 'Y':

                self._client.utils.save_ref_with_inferences(path + op_map)
                self.end = True

            else :
                print("The map is not ready or you have written the wrong character")
                print("Please if the map is ready, write y or Y")

    def reasoner(self):
        print()
        # activate the reasoner 
        self._client.utils.sync_buffered_reasoner()
        self._client.utils.apply_buffered_changes()
        print()

    def can_reach(self):
        return self._client.query.objectprop_b2_ind(self.connection, self.rob)
    
    def urgent(self):
        return self._client.query.ind_b2_class("URGENT")
    
    def corridor(self):
        return self._client.query.ind_b2_class("CORRIDOR")
                
    def get_time(self):

        # take the current time to update the timestamps
        now = str(int(time.time()))

        return now
    
    def update_robot_time(self):

        old_time_now = self._client.query.dataprop_b2_ind(self.now_prop, self.rob)[0]
        old_time_now = re.findall(r'"(.+?)"', old_time_now)[0]
        new_time = self.get_time()
        
        self._client.manipulation.remove_dataprop_from_ind(self.now_prop,self.rob,self.type_value, old_time_now)
        self._client.manipulation.add_dataprop_to_ind(self.now_prop, self.rob, self.type_value, new_time)


    def update_location_time(self, new_loc):

        old_time_vis = self._client.query.dataprop_b2_ind(self.vis, new_loc)[0]
        old_time_vis = re.findall(r'"(.+?)"', old_time_vis)[0]
        new_time = self.get_time()
        
        self._client.manipulation.remove_dataprop_from_ind(self.vis, new_loc, self.type_value, old_time_vis)
        self._client.manipulation.add_dataprop_to_ind(self.vis, new_loc, self.type_value, new_time)

    def update_robot_position(self, old_loc, new_loc):

        self._client.manipulation.remove_objectprop_from_ind(self.rob_prop, self.rob, old_loc)
        self._client.manipulation.add_objectprop_to_ind(self.rob_prop, self.rob, new_loc)

    def ask_rob_pos(self):
        return self._client.query.objectprop_b2_ind(self.rob_prop, self.rob)[0]
    
    def go_in_a_corridor(self,corr,can_reach,goal):
        i = 0
        j = 0
        for i in range(len(can_reach)):
            for j in range(len(corr)):
                if can_reach[i] == corr[j]:
                    #print(Fore.LIGHTRED_EX + "the reachable corridor",Fore.LIGHTYELLOW_EX + " is ",Fore.LIGHTRED_EX + f"{corr[j]} ")
                    goal.append(can_reach[i])
        if goal == [] :
            print()
            print(Fore.LIGHTYELLOW_EX + "NO ",Fore.LIGHTRED_EX + "corridor",Fore.LIGHTYELLOW_EX + " is reachable")
            goal.append(random.choice(can_reach))

    def target_location(self):
        i=0
        j=0

        reachable_urg_loc = []
        goal_loc = []

        rospy.sleep(1)

        # location that the robot can reach 
        reachable_loc = self.can_reach()
        urgent_loc = self.urgent()
        corridors = self.corridor()

        print(Fore.LIGHTYELLOW_EX + "the robot can reach the following locations : ", Fore.WHITE)
        print(*reachable_loc)
        print()
        print(Fore.LIGHTRED_EX + "corridors", Fore.LIGHTYELLOW_EX + " are : ", Fore.WHITE)
        print(*corridors)
        print()

        
        if len(urgent_loc) > 0 :
            print(Fore.LIGHTCYAN_EX + "urgent locations", Fore.LIGHTYELLOW_EX + " are : ", Fore.WHITE)
            print(*urgent_loc)
            print()
            for i in range(len(reachable_loc)):
                for j in range(len(urgent_loc)):
                    if reachable_loc[i] == urgent_loc[j]:
                        reachable_urg_loc.append(reachable_loc[i])
                    
            if len(reachable_urg_loc) > 0:
                print(Fore.LIGHTCYAN_EX + "urgent location", Fore.LIGHTYELLOW_EX + " found ", Fore.WHITE)
                print()
            
            if len(reachable_urg_loc) > 0 :
                print(Fore.LIGHTYELLOW_EX + "Go in the ",Fore.LIGHTCYAN_EX + "urgent location")
                print(Fore.WHITE)
                plan = random.choice(reachable_urg_loc)
            else :
                print(Fore.LIGHTYELLOW_EX + "No ",Fore.LIGHTCYAN_EX + "urgent location", Fore.LIGHTYELLOW_EX + "reachable")
                print()
                print(Fore.LIGHTYELLOW_EX + "Go in a ",Fore.LIGHTRED_EX + "reachable corridor ")
                self.go_in_a_corridor(corridors,reachable_loc,goal_loc)
                print(Fore.LIGHTYELLOW_EX + "The ",Fore.LIGHTRED_EX + "location goal",Fore.LIGHTYELLOW_EX + " is : ")
                plan = random.choice(goal_loc)
                print(plan)
        else :
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
        i = 0
        target = ''

        print()
        print(Fore.LIGHTCYAN_EX + "Searching the charging location...")
        print()
        # location that the robot can reach 
        reachable_loc = self.can_reach()
        corridors = self.corridor()
        actual_loc = self.ask_rob_pos()

        print(Fore.LIGHTYELLOW_EX + "the robot can reach the following locations : ", Fore.WHITE)
        print(*reachable_loc)
        print()

        print(Fore.LIGHTRED_EX + "corridors", Fore.LIGHTYELLOW_EX + " are : ", Fore.WHITE)
        print(*corridors)
        print()
        
        for i in range(len(reachable_loc)):
            if reachable_loc[i] == self.charging_loc[0]:
                target = reachable_loc[i]
                print()
                print(Fore.LIGHTCYAN_EX + "Charging Location found !!!")
                print()
                return Location(target)
        
        i = 0
        j = 0
        reachable_corridors = []

        if len(target) == 0:
            print(Fore.LIGHTYELLOW_EX + "Charging Location not found", Fore.WHITE)
            for i in range(len(reachable_loc)):
                for j in range(len(corridors)):
                    if reachable_loc[i] == corridors[j]:
                        reachable_corridors.append(reachable_loc[i])
        
            if len(reachable_corridors) == 0:
                print(Fore.LIGHTYELLOW_EX + "Go in a random reachable location", Fore.WHITE)
                target = random.choice(reachable_loc)
                return Location(target)
            else:
                print(Fore.LIGHTYELLOW_EX + "Go in a corridor", Fore.WHITE)
                target= random.choice(reachable_corridors)
                return Location(target)

        