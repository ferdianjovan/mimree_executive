# MIMRee Mission Planner Executor

This package is part of the MIMRee project (https://github.com/EEEManchester/MIMRee) that controls the planning and the execution of a mission

## Requirements :
1. Rosplan package (https://github.com/KCL-Planning/ROSPlan).
2. Gazebo version 9.x or greater.
3. Ardupilot.
4. Mavros (`apt-get install ros-melodic-mavros`)
5. pygeodesy (`pip install PyGeodesy`)
6. reportlab (`pip install reportlab==3.2.0`)
7. Tkinter (already installed with Ubuntu 18.04)

## HELP

How to Launch (in conjunction with https://github.com/EEEManchester/MIMRee MIMRee simulator):  

On 1st Terminal (Launch Mavros for both UAV and ASV)
```
roslaunch mimree_executive apm.launch use_asv:=true
```
if the ASV is present in the simulation
```
roslaunch mimree_executive apm.launch use_asv:=false
```
if the ASV is not present

On 2nd Terminal 
```
roslaunch mimree_executive planner.launch
```

On 3rd Terminal (Launch Gazebo on Mangalia world with MIMRee UAV and MIMRee ASV)
```
rosrun mimree_executive mission_executor.py -m mangalia
```
