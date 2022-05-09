### Introduction

Robotic project: A robot that autonomously picks up tennis balls. 

This is the final project of Northeastern University CS5335 Robotics System and Science.

In this project, we aim to design a robot with SLAM, navigation, visual computing, robotic arm grasping functions to help us pick up tennis balls automatically.

In this project, we develop in simulation and physical robot respectively.

The type of robot used in this project is the locobot WX250s, which is a hybrid type robot. 

https://www.trossenrobotics.com/locobot-wx250-6-degree-of-freedom.aspx#documentation

Currently this part of the code belongs to the simulation part.

![locobot2](https://github.com/Yandong-Luo/hybrid/master/image/locobot2.jpg)

#### Result

#### SLAM

对于SLAM部分，我们通过两种方式来完成SLAM，一种是基于RRT算法的自主建图，另一种我们通过键盘控制机器人进行SLAM任务。

##### RRT SLAM

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_rrt_exploration:=true
```

![rrt_exploration4](https://github.com/Yandong-Luo/hybrid/master/image/rrt_exploration4.gif)

##### SLAM Based on Keyboard

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_slam:=true
```

![slam](https://github.com/Yandong-Luo/hybrid/image/image/slam.gif)

#### Navigation

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_navigation:=true
```

![navigation](https://github.com/Yandong-Luo/hybrid/image/image/navigation.gif)

#### Navigation + pick up

Type the following commands in two terminals respectively to achieve the joint task of navigation and grabbing

```
roslaunch interbotix_xslocobot_gazebo locobot_simulation.launch use_moveit:=true use_navigation:=true
```

```
rosrun slam_controller slam_controller_node
```

![pick_nav_decoding 00_00_00-00_00_30](https://github.com/Yandong-Luo/hybrid/image/image/pick_nav_decoding 00_00_00-00_00_30.gif)

#### Pick up Task

If you want to achieve the following functions, you need to adjust the position of the ball in the launch file and modify the content of slam_controller.

![pick_up](https://github.com/Yandong-Luo/hybrid/image/image/pick_up.gif)
