#ifndef SLAM_CONTROLLER_NODE_H_
#define SLAM_CONTROLLER_NODE_H_

#include <iostream>
#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Time.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "interbotix_simulation_controller/MoveItPlan.h"
#include <actionlib_msgs/GoalStatusArray.h>
// #include <interbotix_simulation_controller>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

# define M_PI       3.14159265358979323846  /* pi */

class SlamController
{
private:
    ros::NodeHandle nh_;
    ros::Publisher camera_pub;
    
    // ros::Subscriber clock_sub;

    ros::ServiceClient controller_client;
    
    std::vector<double> home_joint_value = { 0, 0, 0, 0, 0, 0};

    std::vector<double> sleep_joint_value = { 1.55, 0, -1.1, 0, 0.5, 0};

    bool plan_success;

    interbotix_simulation_controller::MoveItPlan srv;

    ros::Subscriber nav_status_sub;

    bool start_pick;
    
public:
    SlamController(ros::NodeHandle* nodehandle);
    
    ~SlamController();

    bool send_angle;
    
    int angle_data;

    bool paused_state;

    void request_set_camera_tilt_angle(float value);

    void request_planning_joint_pose(void);

    void request_execute_moveit_plan(void);
    
    void request_planning_joint_pose_to_sleep(void);

    void request_open_gripper(void);

    void request_pick_up(void);

    void request_navigation(void);

    void nav_status_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& nav_status);

    bool reach_goal;

    // void set_end_effector_pose(moveit::planning_interface::MoveGroupInterface& move_group);
};
SlamController::~SlamController()
{}
#endif