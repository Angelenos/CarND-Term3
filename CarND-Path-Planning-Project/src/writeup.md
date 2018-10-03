# **Path Planning Project**

## Writeup

---

####Build a Path Planner

In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

## Rubric Points
#### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one

You're reading it! and here is a link to my [project code](https://github.com/Angelenos/CarND-Term3/tree/master/CarND-Path-Planning-Project/src)

### Implementation of Planner

**Finite State Machine (FSM)** is adopted here to build the path planner. Also a custom class **Vehicle** is created in line 22 of [vehicle.h][2] to record fundamental properties of a vehicle driving on the highway including speed, position, lane and state. Implementation can be found in [vehicle.cpp][1].

#### 1. States

The FSM of the planner has 5 states

|	State	|	Description											| 
|:-----------:	|:--------------------------------------------:							| 
|	KL		|	Keep current lane										|
|	PLCL	|	Maintain current lane while preparing for left lane change		| 
|	PLCR	|	Maintain current lane while preparing for right lane change	| 
|	LCL		|	Perform left lane change (non-interruptable)				| 
|	LCR		|	Perform right lane change (non-interruptable)				| 

By default the vehicle stays in KL state. PLCL state can be accessed from KL and LCL states. PLCR state can be accessed from KL and LCR states. Both LCL and LCR can only be accessed from PLCL and PLCR, respectively and followed with KL state only. 

In KL, PLCL/PLCR states vehicle will follow the traffic ahead and try to maintain the same speed as the vehicle in front of it. When no traffic presented the vehicle will be driven at 49 mph. However in the PLCL and PLCR states vehicle will also examine the traffic in the left and right lane, respectively. If traffic on either side allows a safe lane switch it will direct into the LCL and LCR states.

In LCL/LCR states, vehicle will perform lane switch. For safety vehicle will enter these states only when PLCL/PLCR confirm sufficient space and safe traffic speed to complete lane switch. Also these states are non-interruptable, which means it will maintain the current state without receiving other possible ones until the vehicle completes the lane switch (see line 86 of [vehicle.cpp][1]). 

#### 2. Costs

State transition is determined by the costs related. Only the possible following state with the lowest cost will be adopted as the next state and used for path generation.

There are 2 types of costs considered here: inefficient cost and lane-switch cost. Inefficient cost is calculated by the velocity difference between the vehicle speed under given state and the target speed (49 mph) divided by the target speed. Lane-switch cost is calculated by the lane difference between the lane under given state and the desired lane (middle lane in this project). Each type of cost will be multiplied by a weighing factors defined in line 20 of [vehicle.h][2]. It is tuned that vehicle only performs lane switch when the traffic ahead will significantly reduce the vehicle speed and vehicle will be shift back to middle lane once possible if no heavy traffic observed.

#### 3. Path Generator

Spline library [spline.h](https://github.com/Angelenos/CarND-Term3/blob/master/CarND-Path-Planning-Project/src/spline.h) is adopted here to generate polynomial fitted path based on the discrete map data to guarantee a smooth path and reduce acceleration and jerks. The function **Vehicle::get_polyfit_path** is defined in line 331 of [vehicle.cpp][1]. Number of points used to perform polyfit and resolutions in sampling can be specified by the caller function.

After the spline object is fed with given map points converted into vehicle local coordinates, it will be used to calculate the next_path vectors. A group of x values in the vehicle local coordiantes are selected based on the future velocity values to calculate the corresponding vehicle local y value, and then transform into the global coordinate as the next_path vectors (line 411 to 451 of [vehicle.cpp][1]).

The planner adopts constant jerk motions during acceleration and deacceleration. When requesting velocity change a constant jerk value (line 14 of [vehicle.h][2]) is added or substrated from the previous acceleration value, then the velocity value to make sure both the acceleration and jerk values within the limit.

### Planner Behavior


#### 1. The car is able to drive at least 4.32 miles without incident

The planner has been tested locally and verified to satisfy the duration requirements

![goal.png][goal]

#### 2. The car drives according to the speed limit

This can be verified by the previous screenshot.

#### 3. Max Acceleration and Jerk are not Exceeded

This can be verified by the previous screenshot.

#### 4. Car does not have collisions

This can be verified by the previous screenshot.

#### 5. The car stays in its lane, except for the time between changing lanes

It can be shown in the previous image that vehicle stays in middle lane when no traffic ahead by default.

#### 6. The car is able to change lanes

When traffic presented ahead and adjacent lanes are available with higher speed, vehicle is able to make change lanes.

![Lane change][lane1]

When middle lane available vehicle is also able to change back to the desired lane

![Lane change][lane2]


[1]: https://github.com/Angelenos/CarND-Term3/blob/master/CarND-Path-Planning-Project/src/vehicle.cpp
[2]: https://github.com/Angelenos/CarND-Term3/blob/master/CarND-Path-Planning-Project/src/vehicle.h
[goal]: img/goal.png
[lane1]: img/lane1.png
[lane2]: img/lane2.png


