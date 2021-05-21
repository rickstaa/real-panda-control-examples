# Real panda control examples

This repository contains a simple example launch file for controlling the physical Panda robot using Moveit. It can be used to check whether the realtime kernel was setup correctly ([see the panda documentation](https://frankaemika.github.io/docs/installation_linux.html)).
 
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

## Launch instructions

After building the catkin package and sourcing the `devel/setup.bash` file. You can launch the example using the following command:

```bash
roslaunch real_panda_moveit_control real_panda_moveit_control.launch
```
