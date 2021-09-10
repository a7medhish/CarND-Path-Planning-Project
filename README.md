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
- Initialization (Macro definition)
- Initialization (variables defined outside JSON message)
- Sensor fusion parsing
- Behavior
- Trajectory build-up
