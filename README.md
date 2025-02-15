# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
# Overview

This project implements a trajectory planner highlighting the criteria under Section [Valid Trajectories](#valid-trajectories).

---

# Rubric Points

## Compilation
The code has been compiled without errors using cmake and make.

## Valid Trajectories
| Criteria | Comment |
| -------- | ------- |
| The car is able to drive at least 4.32 miles without incident. | Finished a lap and no incident popped up. |
| The car drives according to the speed limit. | No speed limit warning message popped up. |
| Max Acceleration and Jerk are not Exceeded. | No acceleration / jerk warning message popped up.|
| Car does not have collisions. | No collisions. |
| The car stays in its lane, except for the time between changing lanes. | The car changes back to its lane whenever safe to do so. |
| The car is able to change lanes. | The car is able to change lane when vehicle exists in front (and too close) and it is safe to change lane. |

## Reflection
There is a reflection on how to generate paths. I've put detailed comments in codes, including but not limited to

### Initialization

Defined constants which are used later in the as macros:
- Velocity converting factor from m/s to mph (2.24)
- Message refreshing period (0.02s) and max path size (50)
- Actual speed limit (49.5mph) and acceleration limit (9.9m/s^2), which are tighter than required in rubrics
- Waypoint step grid (30m)
- Lane parameters (lane width - 4m, default/leftmost/rightmost lane index - 1/0/2)

Also initialized variables for those to be done outside the JSON message:
- (initial) host lane: default lane
- (initial) starting speed: 0m/s (stopped)
- speed incremental step per cycle (max acceleration * message refreshing period)

### Sensor fusion

Parsed the sensor fusion list. If the vehicle is from the host lane or one of the adjacent lane, check if they are "close" -- i.e. within a certain range in Frenet-s direction. The concept "close" was used in behavior logics to determine as accordance of slowing down and/or change lane.

### Behavior

In order to drive safely and also with good performance, the host vehicle was designed with the following behaviors.
| Situation | Action |
| --------- | ------ |
| Other vehicle exists close in front | Switch to left (right) lane if no vehicle exist close in left (right) to make a pass.<br />Otherwise, slow down.|
| Host vehicle is not in default lane | Switch towards the default lane, given there is no vehicle close in the target lane. |
| Host vehicle is too slow | Accelerate up to the speed limit. |

### Trajectory

Firstly, created loose-grid waypoints -- 5 points defined here:
- last 2 points from previous path (when unavailable, use current and calculated "previous" car locations).
- 3 points further in the way (Frenet-s).
Also converted these waypoints into local car coordinates (where reference point's (x, y, yaw) are all 0) for easier spline calculation.

Secondly, calculated trajectory points.
- Use existing trajectory points to keep continuity -- only calculate new points if needed.
- For new points calculation, initialized a spline object using all the waypoints, and calculated new trajectory points (each one 0.02s apart) within maximum 30m ahead. Note the host speed is time-varying -- which was considered in calculation.
- Converted these new points into real-world coordinates.
Then embedded the updated trajectory into JSON message.
