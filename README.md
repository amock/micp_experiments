# micp_experiments

Collection of experiments to evaluate the performance of MICP implemented in [RMCL](https://github.com/uos/rmcl).

## MulRan

- ROS1 (noetic): [micp_mulran](./micp_mulran/)
- ROS2 (humble): [micp_mulran2](./micp_mulran2/)

## Hilti

- ROS1 (noetic): [micp_hilti](./micp_hilti/)

## Convergence

- ROS1 (noetic): [micp_convergence](./micp_convergence/)

## Visualizations

- ROS1 (noetic): Fork of mesh_tools: https://github.com/aock/mesh_tools
- ROS2 (humble): https://github.com/naturerobots/mesh_tools


## Issues

ROS1 recognizes that the build tool is unknown. So it skips all ROS2 packages. ROS2, however, tries to build the ROS1 packages and fails. I tried to place "AMENT_IGNORE" or "COLCON_IGNORE" in every ROS1 package and "CATKIN_IGNORE" in every ROS2 package. But for some reason ROS1 is ignoring "AMENT_IGNORE" packages as well. So: A fix for ROS2 users until I figure out a solution: Place an empty file "AMENT_IGNORE" into every ROS1 package folder.