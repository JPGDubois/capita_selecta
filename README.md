# ROS2 and Gazebo
This project documents how to use Ros2 Humble in conjunction with Gazebo Ignition.

## Stetting up the Development Environment.
A standardised development environment is achieved through the use of devcontainers with vscode. For ubuntu based systems, the following system dependencies are required.
* **vscode**, setup for [dev containers](https://code.visualstudio.com/docs/devcontainers/containers)
* [**docker engine**](https://docs.docker.com/engine/install/ubuntu/)

Clone the git repositry and open it in vscode, a prompt will appear asking to open the project in the dev container. Alternatively, The project can be opened in the dev container by running ```Dev Containers: rebuild and reopen in container``` in the vscode command palette.


## Pathfollowing
This contains a node that performs simple waypoint following missions while avoiding obstacles. The algorithm is not robust and the waypoints may not be accurate to the simulated world due to error buildup in odometry. 

To use it open the gazebo file using:
```
ign gazebo gazebo_files/worlds/pathfollowing.sdf
```

In a new window open the node:
```
colcon build --packages-select pathfollowing --merge-install --symlink-install
```

```
source install/setup.bash
ros2 launch pathfollowing launch_pathfinding.launch.py
```

Then in a third window, a waypoint can be set using the following command:
```
ros2 topic pub --once waypoint geometry_msgs/msg/Pose "{position: {x: 10, y: 0.0, z: 0.0}}"
```



# capita_selecta
