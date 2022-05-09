#include <ros/ros.h>
#include "math.h"
#include "slam_controller/slam_controller_node.h"
#include <std_srvs/Empty.h>

SlamController::SlamController(ros::NodeHandle* nodeHandle):nh_(*nodeHandle)
{
    controller_client = nh_.serviceClient<interbotix_simulation_controller::MoveItPlan>("/locobot/moveit_plan");

    nav_status_sub = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",10,&SlamController::nav_status_Callback,this);

    reach_goal = false;

    start_pick = false;
}

void SlamController::nav_status_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& nav_status)
{
    if (!nav_status->status_list.empty())
    {
        actionlib_msgs::GoalStatus status_info = nav_status->status_list[0];
        uint flag = status_info.status;

        if (flag == 3)
        {
            reach_goal = true;

            if (!start_pick)
            {
                request_pick_up();
                start_pick = true;
            }
        }
    }
}

void SlamController::request_planning_joint_pose()
{
    srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_PLAN_JOINT_POSE;
    
    // {1.55, 0, -1.1, 0, 0.5, 0}
    srv.request.joint_vector = {0,0.5,0,-1.1,0,1.55};

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to planning joint position\n");
    }
}

void SlamController::request_navigation()
{
    geometry_msgs::PoseStamped goal_pose;

    goal_pose.header.frame_id = "map";

    goal_pose.pose.position.x = 5.0;
    goal_pose.pose.position.y = 5.4;
    goal_pose.pose.position.z = 0.0;

    // wheel don't need to rotate the x and y direction
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    // z angle
    float theta = 94.0*M_PI/180.0;

    goal_pose.pose.orientation.z = sin(theta/2);     // 45 degree    
    goal_pose.pose.orientation.w = cos(theta/2);

    srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_NAVIGATION;

    srv.request.nav_goal_pose = goal_pose;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to navigate to the goal point\n");
    }
}

void SlamController::request_planning_joint_pose_to_sleep()
{
    srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_SLEEP;
    
    // {1.55, 0, -1.1, 0, 0.5, 0}
    // srv.request.joint_vector = {0,0.5,0,-1.1,0,1.55};

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to planning joint position\n");
    }
}

void SlamController::request_execute_moveit_plan()
{
    srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_EXECUTE;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to execute moveit plan\n");
    }
}

void SlamController::request_set_camera_tilt_angle(float value)
{
    if (angle_data != value)
    {
        // Update
        angle_data = value;
                
        srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_CONTROL_CAMERA_ANGLE;;
        
        std_msgs::Float64 cmd;

        // convert degree to rad
        // float temp = M_PI*angle_data/180.0f;
        
        cmd.data = value;
        srv.request.camera_angle = cmd;

        if (controller_client.call(srv))
        {
            ROS_INFO("Sum: %s\n", srv.response.msg.data.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service to change tilt angle\n");
        }
    }
}

void SlamController::request_open_gripper(void)
{
    srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_OPEN_GRIPPER;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to open gripper\n");
    }
}

void SlamController::request_pick_up(void)
{
    srv.request.cmd = interbotix_simulation_controller::MoveItPlan::Request::CMD_PICK_UP;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to pick up\n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"slam_controller");

    ros::AsyncSpinner spinner(3);

    spinner.start();
    
    ros::NodeHandle nh;

    SlamController m_slamController(&nh);

    
    m_slamController.request_set_camera_tilt_angle(45.0);

    ros::WallDuration(1.0).sleep();

    m_slamController.request_navigation();

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    
    // m_slamController.request_pick_up();
    


    // ROS_INFO("????????????????????????%d\n",m_slamController.reach_goal);
    // if (m_slamController.reach_goal)
    // {
    //     m_slamController.request_pick_up();
    // }

    // m_slamController.request_planning_joint_pose();
    // m_slamController.request_planning_joint_pose_to_sleep();

    ros::WallDuration(1.0).sleep();

    // m_slamController.request_execute_moveit_plan();

    // ros::WallDuration(1.0).sleep();

    // m_slamController.request_open_gripper();

    ros::WallDuration(1.0).sleep();

    // m_slamController.request_pick_up();

    ros::waitForShutdown();

    return 0;
}