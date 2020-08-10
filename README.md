# real_panda_moveit_control

This repository contains a launchfile example on how to control the physical Panda robot using moveit.

## Build instructions

Install the ROS package dependencies using the following command:

```bash
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```

The catkin package can be build by executing one of the following commands:

```bash
catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build
```

## Launch instructions

After building the catkin package and sourcing the `devel/setup.bash` file. You can launch the example using the following command:

```bash
roslaunch real_panda_moveit_control real_panda_moveit_control.launch
```
