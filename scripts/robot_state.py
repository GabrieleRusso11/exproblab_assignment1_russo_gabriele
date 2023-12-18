#!/usr/bin/env python

"""
.. module:: robot_state
   :platform: Ubuntu 20.04
   :synopsis: Python module which is the shared knowledge among nodes. This knowledge is about the state of the robot

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

This node contains the current robot position, updated by the controller node. This current position is 
then shared with the planner node. The communication with the planner and controller node is implemented 
using two custom Ros services. 
It contains also the state of the robot battery (high or low battery level), and each time there is a change 
of state (from high to low and viceversa) the battery level is published to the state machine node.

ROS Parameters:
  **/battery_time** the waiting time used to simulate the usage of the battery and the recharging time. \n

Service:
  **/state/set_pose** receives from the controller node, the new robot position .\n
  **/state/get_pose** sends to the planner node, the current robot position.\n
  **/inteface/start_charging** receives from the state machine node, the bloolean flag used to handle the charging process. \n

Publisher:
  **/state/battery_low** topic where the robot battery state is published. \n
"""

import threading
import random
import rospy

# Import constant name defined to structure the architecture.
from exproblab_assignment1_russo_gabriele import architecture_names as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from exproblab_assignment1_russo_gabriele.srv import ActualPosition, ActualPositionResponse, NewPosition, NewPositionResponse, StartCharging, StartChargingResponse

# to color the text
import colorama
from colorama import Fore

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE


class RobotState:
    """
    This class defines three services: one to send the current robot position (state/get_pose),
    one to update the robot position (state/set_pose), and one to handle the robot charging process (inteface/start_charging).
    Then defines a publisher to notify the robot battery level (state/battery_low).
    """
    def __init__(self):

        # Initialize this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)

        # initialize the start charging flag
        self._start_charging = False

        # Initialize robot position.
        self._position = None

        # Initialize battery level.
        self._battery_low = False

        # initialize flags
        self.power_on = False
        self.print_flag = False

        # get from the ros parameter server the time used to simulate 
        # the usage of the battery and its recharging time
        self._battery_time = rospy.get_param("/battery_time")
        log_msg = (f'battery notification active: the battery change state (i.e., low/high).')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print()
        
        # Define services.
        # service to get the robot actual position 
        rospy.Service(anm.SERVER_GET_POSE, ActualPosition, self.get_position)
        # service to set the new robot position 
        rospy.Service(anm.SERVER_SET_POSE, NewPosition, self.set_position)
        # service to set the start charging flag
        rospy.Service(anm.SERVER_CHARGING, StartCharging, self.start_charging)

        # Start battery level publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()
        
        # Log information.
        log_msg = (f'Initialize node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}`, '
                   f'`{anm.SERVER_SET_POSE}`, and 'f'{anm.SERVER_CHARGING}' 'and topic 'f'{anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print()
    
    def set_position(self, request):
        """
        This is the 'robot/set_pose' service implementation.
        It takes as 'request' input parameter the new robot position 
        to be set, from the controller node.
        This server returns an empty 'response' (no response is needed).
        """
        # check if the position given by the client is consistent
        if request.position is not None:

            # Store the new current robot position,
            # and set the power_on flag to True.
            self._position = request.position
            self.power_on = True # used in order to start the usage of the battery
                                 # only after the user has created the map,
                                 # therefore only when the robot starts to compute the plan
                                 # for the first time.

            # Log information.
            log_msg = Fore.LIGHTCYAN_EX + f'Set current robot position through `{anm.SERVER_SET_POSE}` 'f'as ({self._position.name}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print(Fore.WHITE)

        else:

            rospy.logerr(anm.tag_log(Fore.LIGHTRED_EX + 'Cannot set an unspecified robot position', LOG_TAG))
            print(Fore.WHITE)

        # Return an empty response.
        return NewPositionResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty.
    # The `response` given to the client contains the actual robot pose.
    def get_position(self, request):
        """
        This is the 'state/get_pose' service implementation.
        The 'request' input parameter is not used, therefore it is empty.
        Then the robot state node, gives as 'response' ,to the client (the planner node)
        the actual robot posisition.
        """
        # check if the position is consistent
        if self._position is None:

            rospy.logerr(anm.tag_log(Fore.LIGHTRED_EX + 'Cannot get an unspecified robot position', LOG_TAG))
            print(Fore.WHITE)

        else:

            log_msg = Fore.LIGHTYELLOW_EX + f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._position.name})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print(Fore.WHITE)

        # Create the response with the actual robot position and return it.
        response = ActualPositionResponse()
        response.position = self._position
        return response

    
    def start_charging(self, request):
        """
        This is the 'inteface/start_charging' service implementation.
        It takes as 'request' input parameter the start charging flag
        used to handle the charging process.
        This server returns an empty 'response' (no response is needed).
        """
        # check if the request (the start flag) is consistent
        if request.start is not None:

            # Store the start charging flag.
            self._start_charging = request.start
            
        else:

            rospy.logerr(anm.tag_log('Error', LOG_TAG))

        # Return an empty response.
        return StartChargingResponse()

    def _is_battery_low(self):
        """
        Publish changes of battery levels when the battery change state (i.e., high/low).
        This method runs on a separate thread.
        The message is published through the `publisher` input parameter and it is 
        a boolean value, i.e., `True`: battery low, `False`: battery high.
        """
        # Publisher intialization.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)

        # initialization of the delay used to simulate the battery usage
        delay_high = self._battery_time 

        # initialization of the delay used to simulate the battery charging 
        delay_low = self._battery_time/10 
        
        while not rospy.is_shutdown():

            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            
            # check if the battery is low and the flag start charging is true.
            # The flag start charging is set to true, by the state machine, only 
            # when the robot is or has reached the charging location.
            if self._battery_low and self._start_charging: 

                print(Fore.LIGHTGREEN_EX + "Charging Location reached")
                print(Fore.WHITE)
                print(Fore.LIGHTGREEN_EX + "Charging the battery")
                print(Fore.WHITE)

                # Wait for simulate battery charging.
                rospy.sleep(delay_low)         

                # Change battery state: Battery level from low to high
                self._battery_low = False                
            
            # instead check if the battery is high and the robot is power on.
            # The power on flag says if the robot is active or it is waiting 
            # the map creation by the user.
            elif (not self._battery_low and self.power_on) : 
                
                if not self.print_flag:
                    
                    # only at beginnig
                    log_msg = 'power on'
                    self.print_flag = True

                else:

                    log_msg = Fore.LIGHTMAGENTA_EX + f'Robot got a fully charged battery after {delay_low} seconds.'

                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                print(Fore.WHITE)

                # Wait for simulate battery usage.
                rospy.sleep(delay_high)    

                # Change battery state: Battery level from high to low
                self._battery_low = True

                log_msg = Fore.LIGHTMAGENTA_EX + f'Robot got low battery after {delay_high} seconds.'
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                print(Fore.WHITE)

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
