# Automatic Landing
This repository is meant to contain all the files that pertain to the Automatic Landing project of the Embedded Systems course.

For more information refer to the [paper](https://drive.google.com/file/d/1KoUr0m3RwfOB22V0X2_huewZzBGtmEw6/view?usp=share_link)

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

## Real world application
For what concerns the running of our algorithm on a real drone, we used the same flight controller (i.e. *pixhawk4 mini*) as for the HITL simulation.
For this reason, all the settings specified in the README.md file in the **HITL_sim** folder can be considered valid also for the application
on the real drone. 

Here are two videos showing the operation of our algorithm in an application with a real drone: [Video 1](https://drive.google.com/file/d/1nGKeoqZEoRYV2-9vChMX1dLas9AyG2GB/view?usp=share_link), [Video 2](https://drive.google.com/file/d/1YOs1qLRiEWdy2g_y97_6qDoTUzlIgySo/view?usp=share_link).
 
