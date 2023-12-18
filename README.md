# Experimental Robotics Laboratory assignment 
This is the first assignment of the university course Experimental robotics laboratory, developed by the student Gabriele Russo

# Introduction
The aim of this assignment is to simulate a surveillance robot in an indoor environment (the pictures below). The robot starts in the E corridor and then it will go to visit the other locations, giving the priority to the urgent locations and to the corridors. The urgent locations are location that the robot hasn't visited for some time. Once it is in a location, it has to wait some time in order to simulate that it is doing something. During this process if the robot gets a low battery level, it has to go in the corridor E which is the charging location.

In order to do this, the indoor environment is associated to an ontology implemented using the [Armor system](https://github.com/EmaroLab/armor) and the robot behaviour is defined using a Final State Machine which is implemented through [ROS SMACH](http://wiki.ros.org/smach).

The code of this assignment has been developed using as landmark the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) by [Professor Luca Buoncomapgni](https://github.com/buoncubi).

![Assignment environment](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/assignment_map.png)

# Software Architecture
This software architecture is composed of four python scripts:

- `state_machine.py`
- `robot_state.py`
- `planner.py`
- `controller.py`

Then there are other two python scripts:

- `state_machine_interface.py` : which is an interface for the `state_machine.py` in order to communicate with the others nodes, and it also contains many methods used in the state machine node to compute data and to interact with the ontology

- `architecture_names.py` : contains all the names of the nodes and the names of topics, services and actions.

In order to explain in efficient way the software architecture, in the following, are exploited three diagrams :

- **Component diagram**: explain the structure of the architecture and the topics, services and actions that are used

- **State diagram** : explain the structure of the Finite State Machine node

- **Temporal diagram** : explain the temporal workflow of the architecture  

## Component diagram

![Component diagram](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/assignment1_component_diagram.png)

The architecture is composed of 5 components:

- **Armor server**: this component is the interface between the state machine node and the ontology associated to the indoor environment.
It is implemented by the [Armor system](https://github.com/EmaroLab/armor).
In order to load the ontology, do changes and queries, is used the [ArmorPy API](https://github.com/EmaroLab/armor_py_api).

- **State Machine**: this component is the heart of the architecture. It handles all the other nodes in order to define the robot behaviour in the most efficient way.
It is implemented using the [ROS SMACH](http://wiki.ros.org/smach), and has an interface (`state_machine_interface.py`) used to communicates with the other nodes (components) and also to contains methods used to compute data and to interact with the ontology. (see the state diagrams to understand the composition of this component).

- **Robot State**: this component contains the state of the robot, so its actual position and its battery level.

- **Planner**: this component is an action server that takes the actual robot position by the Robot State component and the target position by the State Machine component and returns, to the State Machine, the plan that the robot has to follow in order to reach the target location.

- **Controller**: this component is anc action server that takes the plan by the state machine and simulate the robot motion. Then it sends the new location reached to the State Machine and Robot State components.



## State diagram

![State diagram](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/state_diagram_1.png)

The State machine componet is described by this state diagram.
The state machine is composed by 5 states:

- **WaitForTopMap**: in this state the robot waits that the user has built and loaded the ontology through the armor server. So, in a nutshell waits that the map of the indoor environment is ready.

- **Planning**: once the map is ready, the robot passes to the planning state. In this state the robot plans in which location has to go, obtaining the plan thanks to the planner node.

- **Moving**: in this state is simulated the motion of the robot towards the target location, using the controller node.

- **Wait**: when the robot has reached the target location it waits some times. So in this state is simulated through a busy waiting, that the robot is doind something in the reached location.

- **Recharging**: in this state the robot has a low battery level, so starts to reach the charging location. When it is in the charging location it starts the charging process.


## Temporal diagram

![temporal diagram](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/temporal_diagram_1.png)

This diagram explain the temporal workflow of the architecture.
Firstly, the state machine allows to the user to create the map and load it in the Armor server.
Secondly, the state machine sends the initial position to the robot state node.
Thirdly, the planner node receives the current robot position by the robot state node and the target location by the state machine nodes and returns the plan to the state machine node.
Then, the state machine (in the moving state) sends the plan to the controller node, which simulates the motion sending the reached locations to the robot state node and then the final location to the state machine node.
Lastly the state machine node, through the armor server, update the ontology with the new robot position and with the new robot and location timestamps.
For simplicity the charging process is not shown in the temporal diagram. Obviously, the robot state sends every time the battery level to the state machine. When the battery has a low level the state machine checks if the robot is already in the charging location, if not it search the charging location using the same mechanism with the planner and controller nodes, already shown in the picture above. 

## list of ros parameters

- `starting_map`: is the file used as ontology base

- `operative_map`: is the final ontology file

- `charging station`: indicates the charging location
- `location_property`: the name of the property that adds doors to the locations in the ontology

- `robot_property`: the name of the property that set and get the robot position in the ontology

- `urgent_property`: the name of the property that sets the urgency threshold of a location

- `visited_property`: the name of the property that updates the location timestamp

- `canreach_property`: the name of the property that gets the locations which the robot can reach

- `now_property`: the name of the property that updates the robot timestamp

- `robot_name`: the name of the robot

- `waiting_time`: the time used for the busy wainting in the wait state

- `planning_time`: the time used to simulate the planning computational time

- `moving_time`: the time used to simulate the robot motion

- `battery_time`: the time used to define the charging and discharging time of the battery

- `charging_location`: indicates the name of the charging location

# Installation and running procedure.
This simulation run on Ros Noetic.
Firstly, to run this project is needed the [Armor system](https://github.com/EmaroLab/armor) and the [ROS SMACH](http://wiki.ros.org/smach)
In order to install the Armor, follow this [Tutorial](https://github.com/EmaroLab/armor/issues/7).

To install Ros Smach for the final state machine : 

`sudo apt-get install ros-noetic-executive-smach*`

Then in the workspace src folder clone this repository : 

`git clone https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele.git`

and the repository with the starting ontology:

`git clone https://github.com/buoncubi/topological_map`

In order to run this project, go in the workspace and launch the launch file:

`roslaunch exproblab_assignment1_russo_gabriele assignment1.launch`

# Video Demo.

# Working hypothesis and environment 
For this project are considerated different assumptions.
Firstly, the environment is considered as a 2D map, so the robot doesn't interact with a real simulation environment.
Then, the robot starts from the E corridor which is both the charging location and a normal location to be visited.
Then, there is a priority in the choice of the location to visit. Robot has to visit before the corridors and then the rooms.
If there are no corridors between the urgent locations to visit, then the robot will give higher priority to the urgent room, otherwise the corridors have always the higher priority.
In this way, since the corridors are connected to more locations (a corridors has to have at least 2 doors) than the rooms, the robot can reach more location and can reach quickly the charging location when the battery is low.
The battery level low is considered has a minimum battery level threshold after that the robot has to charge is battery as soon as possible. 
When the battery level is low, if the robot is already in the charging location, the charging process starts immediatly, otherwise the robot is in the low battery level mode and the planner and the controller will be used only to find and reach the charging location.
So when the robot is in the rescharging state, it can go in the plannig state both if the battery is charged and also if the robot has to reach the charging location (low battery mode)

 
## System’s features

In the following are listed the system's features:

- Colored User Interface in order to allow the users to define and create the indoor environment as they prefer.

- High priority to battery_low signals in order to recharge the robot's battery as soon as possible

- possibilty of setting the parameters saved in the ROS parameter server for a tailored functioning of the system and for passing the ontology folder path from the launch file

- python script to manage the ontology and interact with it and to function as interface between the state machine node and the other nodes

- robot_state node that keeps track of the robot state (position and battery level)

- SImple algorithms for planning and controlling the robot (simulating the necessary time for performing the corresponding operations)

- Log informations for each node with different colors

## System’s limitations and Possible technical Improvements

- **Environment**: in this assignment there is an ideal environment, so there aren't walls and obstacle.
So a first possible improvement is to build a more realistic simulated environment for instance with walls and obstacle, in order to test the behaviour of the code and of the robot in a more proper way.

- **Planner and Controller**: the planner and the controller node are very simple and they do nothing of practical. The planner simply creates a plan composed by the initial position, a generic intermediate position and the target position. The controller simply takes the plan and update the robot position until it reaches the final location. So both nodes simulates a planner and controller nodes. A possible improvement is to develop a more realistic and efficient planner and controller algorithms, for instance to evaluate a reasonable path towards the target location (avoiding obstacles and walls) and to control the actuator of the robot in order to follow the generated path.

- **Battery management**: Actually the battery level is only a boolean variable that passes from True to false (and viceversa) in order to simulate the change of the battery level. The charging and discharging of the battery are a simple busy waiting times.
So a first improvement is to output the charging and the discharging of the battery in order to show to the users the increasing or decreasing of the battery. Lastly, the battery management can be improved with a more realistic and proper algorithm, in order to make the battery usage more realistic.

- **Urgent Locations management**: the urgent location to visit is chosen randomly between the locations in the urgent location list (if there are corridors, the priority is given to them), this means that there is not an order based on the time (for example first urgent location to visit is the location that the robot has not visited in the longest time). So a possible improvement is to handle the urgent location list giving higher priority based on the less recent visited time.


# Authors and contacts (at least the email).

**Institution**: University of Genoa

**Course**: MSc in Robotics Engineering

**Subject**: Experimental Robotics Laboratory

**Author**: Gabriele Russo

**Student Id**: S5180813

**E-mail**: gabriele.russo117@gmail.com