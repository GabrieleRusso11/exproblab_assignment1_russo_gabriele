# Experimental Robotics Laboratory assignment 
This is the first assignment of the university course Experimental robotics laboratory, developed by the student Gabriele Russo

# Introduction
The aim of this assignment is to simulate a surveillance robot in an indoor environment (the pictures below). The robot starts in the E corridor and then it will go to visit the other locations, giving the priority to the urgent locations and to the corridors. The urgent locations are location that the robot hasn't visited for some time. Once it is in a location, it has to wait some time in order to simulate that it is doing something. During this process if the robot gets a low battery level, it has to go in the corridor E which is the charging location.

In order to do this, the indoor environment is associated to an ontology implemented using the [Armor system](https://github.com/EmaroLab/armor) and the robot behaviour is defined using a Final State Machine which is implemented through [ROS SMACH](http://wiki.ros.org/smach).

The code of this assignment has been developed using as landmark the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) by [Professor Luca Buoncomapgni](https://github.com/buoncubi).

![Assignment environment](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/assignment_map.png)

# Software Architecture
## component diagram

![Component diagram](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/assignment1_component_diagram.png)

## state diagram

![State diagram](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/state_diagram_1.png)

## temporal diagram

![temporal diagram](https://github.com/GabrieleRusso11/exproblab_assignment1_russo_gabriele/blob/main/media/temporal_diagram_1.png)

## list of ros messages and parameters

# Installation and running procedure(including all the steps to display the robot’s behavior).

# A commented small video, a GIF, or screenshots showing the relevant parts of the running code.

# Working hypothesis and environment (1 or 2 paragraph).
## System’s features (1 or 2 paragraph).
## System’s limitations (1 or 2 paragraph).
## Possible technical Improvements (1 or 2 paragraph).

# Authors and contacts (at least the email).

**Institution**: University of Genoa

**Course**: MSc in Robotics Engineering

**Subject**: Experimental Robotics Laboratory

**Author**: Gabriele Russo

**Student Id**: S5180813

**E-mail**: gabriele.russo117@gmail.com