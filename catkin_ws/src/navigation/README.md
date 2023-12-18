# Exploration and Navigation:
The main goal of this part is to explore the racetrack and navigate the drone through the gates by generating waypoints between each two pylons. 
## 0. Calibration:
The drone can be calibrated by calculating the previous position of the drone and the current position in `calibration.cpp` . This step is done automatically at the beginning of the simulation to ensure that the drone looks at the right direction.

## 1. Exploration:
The racetrack can basicly be explored using two search strategies:
1. Step Forward Search: This is a simple search strategy where the drone moves forward and checks if there is a pylon in front of it. If there is a pylon, the drone moves to the pylon and then moves forward again.
2. Step Around Search: This strategy concerns with the fact that the drone can have a very sharp left-curve and only one of the pylons can be detected. The drone can do backward step and allign itself with the pylon and then move forward again.

## 2. Navigation:
The waypoints are than used to generate a path for the drone to be followed using `MultiDOFJointTrajectory` library

The message is published to the topic `/command/trajectory` and the drone follows the path.