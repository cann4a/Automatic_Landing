# Automatic Landing
The aim of this work was to develop an automatic landing infrastructure suited for the OpenMV Camera H7 Plus microcontroller.  
The final objective of this project was to obtain a totally independent application that runs on the OpenMV Camera without any assistance from external libraries for the drone control part.  
The final application was tested both in a simulation using a SITL (software in the loop) and a HITL (hardware in the loop) framework. For the latter, a real flight controller was used in view of 
a final real-world application. A real-world testing was finally performed.  

This repository contains all the material used for building the automatic landing infrastracture. 


For more information refer to the [paper](https://drive.google.com/file/d/1qEq37-8m7lBh5_CL3vyxKU9pqA7KdrMg/view?usp=share_link)

## Content
This repository contains the following folders:
  - **SITL_sim**: Contains the python program, the files and the instructions necessary to replicate the SITL simulation
  - **HITL_sim**: Contains the main python file run by the OpenMV cam, the NN file for the inference, as well as all the instruction for the
  replication of the HITL simulation
  
## Hardware and Software prerequisites
The following hardware was used for the project development:
  - Pixhawk4 mini flight controller
  - OpenMV Cam H7 Plus + uSD
  - QAV250 airframe kit

The software used was:
  - QGroundControl 
  - OpenMV IDE
  - Gazebo simulator
  - PX4 autopilot
  
  All sofware versions and the python packages list are listed in the file <code>software_version</code>.

## Real world application
For what concerns the running of our algorithm on a real drone, we used the same flight controller (i.e. *pixhawk4 mini*) as for the HITL simulation.
For this reason, all the settings specified in the README.md file in the **HITL_sim** folder can be considered valid also for the application
on the real drone. 

Here are two videos showing the operation of our algorithm in an application with a real drone: [Video 1](https://drive.google.com/file/d/1nGKeoqZEoRYV2-9vChMX1dLas9AyG2GB/view?usp=share_link), [Video 2](https://drive.google.com/file/d/1YOs1qLRiEWdy2g_y97_6qDoTUzlIgySo/view?usp=share_link).
 
