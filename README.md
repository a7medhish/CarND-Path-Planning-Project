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
| The car is able to drive at least 4.32 miles without incident. | |
| The car drives according to the speed limit. | |
| Max Acceleration and Jerk are not Exceeded. | |
| Car does not have collisions. | |
| The car stays in its lane, except for the time between changing lanes. | |
| The car is able to change lanes. | |

## Reflection
There is a reflection on how to generate paths. I've put detailed comments in codes, including but not limited to
- Initialization (Macro definition)
- Initialization (```lane``` and ```ref_vel```)
- Sensor fusion parsing
- Behavior
- Trajectory build-up
