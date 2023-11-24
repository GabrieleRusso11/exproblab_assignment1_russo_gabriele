#!/usr/bin/env python

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

# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    def __init__(self):
        # Initialise this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)

        self._start_charging = False

        # Initialise robot position.
        self._position = None

        # Initialise battery level.
        self._battery_low = False
        self.power_on = False
        self.print_flag = False

        self._battery_time = rospy.get_param("/battery_time")
        log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high).')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, ActualPosition, self.get_position)
        rospy.Service(anm.SERVER_SET_POSE, NewPosition, self.set_position)
        rospy.Service(anm.SERVER_CHARGING, StartCharging, self.start_charging)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()
        
        # Log information.
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # The `robot/set_pose` service implementation.
    # The `request` input parameter is the current robot pose to be set,
    # as given by the client. This server returns an empty `response`.
    def set_position(self, request):
        if request.position is not None:
            # Store the new current robot position.
            self._position = request.position
            self.power_on = True
            # Log information.
            self._print_info(Fore.LIGHTCYAN_EX + f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                             f'as ({self._position.name}).')
            print(Fore.WHITE)
        else:
            rospy.logerr(anm.tag_log(Fore.LIGHTRED_EX + 'Cannot set an unspecified robot position', LOG_TAG))
            print(Fore.WHITE)
        # Return an empty response.
        return NewPositionResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty. Thus, it is not used.
    # The `response` returned to the client contains the current robot pose.
    def get_position(self, request):
        # Log information.
        if self._position is None:
            rospy.logerr(anm.tag_log(Fore.LIGHTRED_EX + 'Cannot get an unspecified robot position', LOG_TAG))
            print(Fore.WHITE)
        else:
            log_msg = Fore.LIGHTYELLOW_EX + f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._position.name})'
            self._print_info(log_msg)
            print(Fore.WHITE)
        # Create the response with the robot pose and return it.
        response = ActualPositionResponse()
        response.position = self._position
        return response

    def start_charging(self, request):
        if request.start is not None:
            # Store the new current robot position.
            self._start_charging = request.start
            
        else:
            rospy.logerr(anm.tag_log('Error', LOG_TAG))
        # Return an empty response.
        return StartChargingResponse()

    # Publish changes of battery levels. This method runs on a separate thread.
    # Publish when the battery change state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def _is_battery_low(self):
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        delay_high = self._battery_time 
        delay_low = self._battery_time/10 
        
        while not rospy.is_shutdown():

            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            
            if self._battery_low and self._start_charging: # if the battery is low
                print(Fore.LIGHTGREEN_EX + "Charging Location reached")
                print(Fore.WHITE)
                print(Fore.LIGHTGREEN_EX + "Charging the battery")
                print(Fore.WHITE)
                # Wait for simulate battery charging.
                rospy.sleep(delay_low)         
                # Change battery state.
                self._battery_low = False                
            
            elif (not self._battery_low and self.power_on) : # if the battery is high
                if not self.print_flag:
                    log_msg = 'power on'
                    self.print_flag = True
                else:
                    log_msg = Fore.LIGHTMAGENTA_EX + f'Robot got a fully charged battery after {delay_low} seconds.'
                self._print_info(log_msg)
                print(Fore.WHITE)
                # Wait for simulate battery usage.
                rospy.sleep(delay_high)         
                # Change battery state.
                self._battery_low = True
                log_msg = Fore.LIGHTMAGENTA_EX + f'Robot got low battery after {delay_high} seconds.'
                self._print_info(log_msg)
                print(Fore.WHITE)

    # Print logging only when random testing is active.
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def _print_info(self, msg):
        rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()