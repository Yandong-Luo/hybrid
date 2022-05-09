机器人项目：自主拾取网球的机器人

这是Northeastern Universitt CS5335 Robotics System and Science的final project。

在这个项目中，我们旨在设计一个具备SLAM，导航，视觉计算，机械臂抓取功能的机器人，以帮助我们自动拾取网球。

这一项目所使用的机器人类型是locobot WX250s，这是一个hybrid类型的机器人。

### Introduction

Robotic project: A robot that autonomously picks up tennis balls. 

This is the final project of Northeastern University CS5335 Robotics System and Science.

In this project, we aim to design a robot with SLAM, navigation, visual computing, robotic arm grasping functions to help us pick up tennis balls automatically.

In this project, we develop in simulation and physical robot respectively.

Currently this part of the code belongs to the simulation part.

#### Result

#### SLAM

对于SLAM部分，我们通过两种方式来完成SLAM，一种是基于RRT算法的自主建图，另一种我们通过键盘控制机器人进行SLAM任务。

##### RRT SLAM

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_rrt_exploration:=true
```

##### SLAM Based on Keyboard

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_slam:=true
```



![rrt_exploration4](.....\final project\presentation\video\simulation\rrt_exploration4.gif)

#### Navigation

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_navigation:=true
```



#### Navigation + pick up

分别在两个终端中进行这个可以实现导航和抓取的联合任务

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_moveit:=true use_navigation:=true
```

```
rosrun slam_controller slam_controller_node
```