# **Path Planning Project** 

## Writeup

---

**In this project, the goal was to implement a highway-driving model that generates
jerk-minimal trajectories following a simple lane-changing state machine consisting
of 3 macro states: "KEEP_LANE", "PREP_CL/CR", "CHANGE_LEFT/RIGHT"**


[//]: # (Image References)

[image1]: ./figures/training-class-examples.png "Training Examples"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. The code compiles correctly.

The code does successfully compile without errors.

### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident..

The car can drive at least 4.32 miles without incident.

#### 2. The car drives according to the speed limit.

The car drives just under the maximum speed limit (50mph) at 49.5mph.

#### 3. Max Acceleration and Jerk are not Exceeded. 

Max acceleration is not exceeded in lateral nor longitudinal directions.
All trajectories are jerk minimal and never exceed the Jerk limit.

#### 4. Car does not have collisions.

The car does not collide with other cars in its own lane or while changing lanes.

#### 5. The car stays in its lane, except for the time between changing lanes.

The car stays in its lane until it deems it appropriate to switch lanes.

#### 6. The car is able to change lanes

The car triggers lane-changing behavior after it approaches a car in its own lane
within a certain distance (40m) so long as it finds a lane it can safely switch to.

### Reflection

#### 1. There is a reflection on how to generate paths.

My implementation generates paths using a stateful approach: main.cpp constructs a 
callback-based system connecting my stateful Planner class with the simulator. Periodic
callbacks feed sensor output from the simulator as input to the Planner, which computes
trajectories according to the behavior designated by its internal state.
These trajectories then are sent back to the simulator.
The trajectories are of 3 major classes (one for each macro state or "behavior" of the Planner):
- KEEP_LANE: the velocity maintained is the maximum allowed by the lane (i.e. the 
        speed limit or the speed of the car ahead of ego-car in this lane), and
        the lateral position is kept close to the center of the current lane.
- PREP_* (the state before switching lanes): velocity is reduced until a threshold
  distance is reached relative to the car ahead in the ego-lane. Once far enough away
  from the car ahead, trajectories will match the speed of car ahead to maintain 
  this safe distance prior to the lane-change behavior. The car's lateral position
  is kept the same as in KEEP_LANE trajectories.                                             
- CHANGE_*: velocities are determined according to the speed(s) of car(s) in the adjacent
  lane that the ego-car is changing to. If the target lane has 2 cars (a "lead" and a "follow")
  then the "merge speed" (aka the ego-car's new target speed) is the average of 
  these 2 cars' speeds. If only one car is in the target lane, then ego-car will 
  match this cars speed while switching lanes. Otherwise, the ego-car will be allowed 
  to accelerate to the speed limit if there are no cars in target lane.
  Of course, trajectories of this class interpolate the lateral position to switch 
  from the center of current lane to the center of the target lane.

How trajectories are prepared:
At the core of my implementation, a buffer of 50 trajectory points is shared between the 
simulator and the Planner. "New trajectories" are more accurately described as
additional trajectories points added to the end of the trajectory buffer as the 
Planner state and the state of the other cars on the road is updated periodically.
Trajectory points are simply XY-points separate by a constant time interval that
belong to jerk-minimal (i.e. 2nd derivatives are 0) trajectories prepared using 
the cubic spline interpolation header-only library ('spline.h').

Note on velocity control: in order to generate trajectories that obey maximum 
total acceleration, the ego-car does not instantaneously adopt the new target speed
for every new trajectory points. Instead, the current speed is made to track
the target speed by incrementally accelerating/decelerating by a constant value.
In cases where the ego-car is changing lanes, this incremental acceleration constant
value is greatly discounted (i.e. the car accelerates 3x's slower when changing lanes)
in order to account for the contribution of lateral acceleration to the car's total
acceleration.

Lane changing conditions:
Lane changes only occur after the ego-car has approached a car in its lane from the rear
within a threshold distance and the other car is slower than the speed limit.
Once this occurs, a target lane is decided to be that lane with the fastest
traffic speed (which is the speed limit if the lane is "empty") - if both left and
right lanes have the same traffic speed, there is a bias towards the left lane.
Once a target lane is decided, the Planner state-machine enters the lane-change 
preparation state for direction of the lane change: i.e. PREP_CL for left lane
or PREP_CR for right lane. In the preparation state, the ego-car will slow down
as mentioned above to maintain a safe distance from the car ahead in its lane 
while it waits for an opening in the target lane. The conditions for an "opening"
are defined in the "CHANGE_*" macro-states description above.
