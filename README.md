# For Go2 (Todo)

Used robot descriptions from https://github.com/khaledgabr77/unitree_go2_ros2/tree/main/unitree_go2_description
and https://github.com/sparcs-unipd/open_manipulator_X/tree/main/open_manipulator_x_description.

# Webots ROS2 Spot

[![ROS2 Humble](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml/badge.svg?branch=main)](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml)

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/). Spot is able to walk around, to sit, standup and lie down. We also attached some sensors on spot, like a kinect and a 3D laser.
The world contains apriltags, a red line to test lane follower and objects for manipulation tasks.

![Spot](https://github.com/MASKOR/webots_ros2_spot/blob/main/spot.jpg)

## Prerequisites

    - Ubuntu 22.04
    - ROS2 Humble https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    - Webots 2025a https://github.com/cyberbotics/webots/releases/tag/R2025a

## Install

1. Install ROS2 Development tools and initialise and update rosdep:
    ```
    sudo apt install -y ros-dev-tools
    ```
    ```
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    ```

2. Create a new ROS2 workspace:
    ```
    export COLCON_WS=~/ros2_ws
    mkdir -p $COLCON_WS/src
    ```

3. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```
    cd $COLCON_WS
    git clone https://github.com/MASKOR/webots_ros2_spot src/webots_ros2_spot
    rosdep install --ignore-src --from-paths src -y -r
    vcs import --recursive src --skip-existing --input src/webots_ros2_spot/webots_ros2_spot.repos
    chmod +x src/webots_ros2/webots_ros2_driver/webots_ros2_driver/ros2_supervisor.py
    ```

4. Build packages and source the workspace
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```

## Start
Starting the simulation:
```
ros2 launch webots_go2 go2_launch.py
```

To launch navigation with Rviz2:
```
ros2 launch webots_go2 nav_launch.py set_initial_pose:=true
```

To launch mapping with Slamtoolbox:
```
ros2 launch webots_go2 slam_launch.py
```

Starting MoveIt:
```
ros2 launch webots_go2 moveit_launch.py
```

Teleop keyboard:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# OR ros2 run spot_teleop spot_teleop_keyboard for body_pose control as well
```

## To switch Arenas

1) Change false to true in https://github.com/MASKOR/webots_ros2_spot/blob/main/resource/spot_control.urdf#L5

2) Change map.yaml to map_arena3.yaml https://github.com/MASKOR/webots_ros2_spot/blob/main/launch/nav_launch.py#L15 (map of arena 2 not created)
