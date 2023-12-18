# Trajectory Generation

This module is responsible for launching the 6-dimensional trajectory package.

## 6-dimensional Trajectory Optimization

The [mav_trajectory_generation][mav_trajectory_generation] package is used which only allows for 3-dimensional or 4-dimensional trajectories. To deal with this we use the method `getTrajectoryWithAppendedDimension` to expand a trajectory for position with a trajectory for rotation. It is publishing two different topics: **/trajectory_markers** (which publishes the marker array of the trajectory) and **/trajectory** (which publishes the trajectory poynomial).

After each lap the list of waypoints used is updated and the trajectory is recalculated. 

The waypoints are updated from the service provided by the **map_generation** module. 

## Benchmarking

The benchmarking is done by the **timing** module. The average value for a 10 lap race was calculated to be 41,21 seconds.

# Reference Links

- [mav_trajectory_generation][mav_trajectory_generation]

[mav_trajectory_generation]: https://github.com/ethz-asl/mav_trajectory_generation
