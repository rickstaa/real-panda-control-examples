# Real panda control examples

This repository contains several examples for controlling the real panda robot. It was created as a supliment to the official[panda documentation](https://frankaemika.github.io/docs/installation_linux.html). Further it serves as a storage place for several problems I encountered while working with the panda robot (see the [discussions section](https://github.com/rickstaa/real-panda-control-examples/discussions)).
 
## Clone instructions

To clone the respository use the following command:

```bash
mkdir real_catkin_ws
cd real_catkin_ws
git clone --recurse-submodules https://github.com/rickstaa/real_panda_moveit_control.git src
```

## Build instructions

Install the ROS package dependencies using the following command:

```bash
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```

The catkin package can be build by executing one of the following commands:

```bash
catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

## Franka ros examples

Please see [this discussion post](https://github.com/rickstaa/real-panda-control-examples/discussions/4) that explains how to run the example launch files provided by Emika Franka.

## Moveit example launch instructions

To test out Moveit control, after you build and sourced the catkin workspace, you can you can launch the example using the following command:

```bash
roslaunch real_panda_moveit_control real_panda_moveit_control.launch
```
