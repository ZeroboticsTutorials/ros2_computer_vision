# ROS2 computer vision

## 1. CV bridge tutorial

### Installation setup

Install cv-bridge:

```shell
sudo apt-get install ros-jazzy-cv-bridge
```

### Test the nodes

```shell
# C++ node
ros2 run cv_bridge_tutorial cvbridge_example_cpp
```

```shell
# Python node
ros2 run cv_bridge_tutorial cvbridge_example_py.py
```

To visualize the results

```shell
# Start rviz2 with config file from the tutorial
rviz2 -d src/cv_bridge_tutorial/rviz/cvbridge_tutorial.rviz
```
