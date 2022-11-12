# Automatic Landing
This repository is meant to contain all the files that pertain to the Automatic Landing project of the Embedded Systems course.

## Content
This repository contains the following folders:
  - **SITL_sim**: Contains the python program, the files and the instructions necessary to replicate the SITL simulation
  - **HITL_sim**: Contains the main python file run by the OpenMV cam, the NN file for the inference, as well as all the instruction for the
  replication of the HITL simulation
  
## Real world application
For what concerns the running of our algorithm on a real drone, we used the same flight controller (i.e. *pixhawk4 mini*) as for the HITL simulation.
For this reason, all the settings specified in the README.md file in the **HITL_sim** folder can be considered valid also for the application
on the real drone. 

Here are two videos showing the operation of our algorithm in an application with a real drone: [Video 1](https://drive.google.com/file/d/1JV9n3LqHvyLHUtkBabWbBYrUyyVq6EYb/view?usp=sharing), [Video 2](https://drive.google.com/file/d/1JO3kJ0CU_yckwGp1dTDTROPrFzG-gI-1/view?usp=sharing).
 
