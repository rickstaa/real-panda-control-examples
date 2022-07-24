# Real panda control examples

This repository contains several examples for controlling the real panda robot. It was created as a supplement to the official [panda documentation](https://frankaemika.github.io/docs/installation_linux.html). Further it serves as a storage place for several problems I encountered while working with the panda robot (see the [discussions section](https://github.com/rickstaa/real-panda-control-examples/discussions)).

## Clone instructions

To clone the repository use the following command:

```bash
mkdir real_catkin_ws
cd real_catkin_ws
git clone --recurse-submodules https://github.com/rickstaa/real_panda_control_examples.git src
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

To test out Moveit control, after you build and sourced the catkin workspace, you can launch the example included in the `panda_moveit_config` using the following command:

```bash
roslaunch panda_moveit_config panda_control_moveit_rviz.launch load_gripper:=true robot_ip:=172.16.0.2
```

Additionally the `real_panda_control_examples` contains a slightly modified version of this example:

```bash
roslaunch real_panda_control_examples real_panda_moveit_control.launch
```

## Trajectory control example launch instructions

To test out Trajectory control, after you build and sourced the catkin workspace, you can launch the example included in the `panda_moveit_config` using the following command:

```bash
roslaunch panda_moveit_config panda_control_moveit_rviz.launch load_gripper:=true robot_ip:=172.16.0.2
```

Additionally the `real_panda_control_examples` contains a slightly modified version of this example:

```bash
roslaunch real_panda_control_examples real_panda_traj_control.launch
```

## Contributing

Feel free to open an issue if you have ideas on how to make this GitHub action better or if you want to report a bug! All contributions are welcome. :rocket: Please consult the [contribution guidelines](CONTRIBUTING.md) for more information.
