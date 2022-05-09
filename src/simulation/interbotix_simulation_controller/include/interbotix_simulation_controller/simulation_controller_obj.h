#ifndef _SIMULATION_CONTROLLER_OBJ_H_
#define _SIMULATION_CONTROLLER_OBJ_H_

#include "interbotix_simulation_controller/MoveItPlan.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// const double tau = 2 * M_PI;

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

class SimulationController
{
public:
    explicit SimulationController(ros::NodeHandle *node_handle);

    bool publish_camera_angle(const std_msgs::Float64 angle_data);

    bool plan_end_effector_pose(const geometry_msgs::Pose pose);

    bool plan_joint_positions(const std::vector<double> joint_group_positions);

    bool plan_joint_pose_to_home(void);

    bool plan_joint_pose_to_sleep(void);

    bool execute_plan(void);

    // gripper
    bool open_gripper(void);

    bool close_gripper(void);

    geometry_msgs::Pose moveit_get_end_effector_pose(void);

    bool pick_up(void);

    void add_tennis_ball_as_object(void);

    void closedGripper(trajectory_msgs::JointTrajectory& posture);

    void openGripper(trajectory_msgs::JointTrajectory& posture);

    bool publish_navigation_goal(const geometry_msgs::PoseStamped goal_pose);

    /// @brief Destructor for the SimulationController
    ~SimulationController();
private:
    ros::NodeHandle node;
    ros::ServiceServer srv_moveit_plan;
    ros::Publisher camera_pub;

    ros::Publisher nav_goal_pub;
    
    const robot_state::JointModelGroup *arm_joint_model_group;
    moveit::planning_interface::MoveGroupInterface *arm_move_group;

    const robot_state::JointModelGroup *gripper_joint_model_group;
    moveit::planning_interface::MoveGroupInterface *gripper_move_group;

    moveit::planning_interface::MoveGroupInterface::Plan saved_plan;  

    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    bool Command_Process(interbotix_simulation_controller::MoveItPlan::Request &req, interbotix_simulation_controller::MoveItPlan::Response &res);
};

#endif