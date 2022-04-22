#include <ros/ros.h>
#include "math.h"
#include "slam_controller/slam_controller_node.h"
#include <std_srvs/Empty.h>

SlamController::SlamController(ros::NodeHandle* nodeHandle):nh_(*nodeHandle)
{

    controller_client = nh_.serviceClient<interbotix_simulation_controller::MoveItPlan>("/locobot/moveit_plan");

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

    ros::AsyncSpinner spinner(2);

    spinner.start();
    
    ros::NodeHandle nh;

    SlamController m_slamController(&nh);

    m_slamController.request_set_camera_tilt_angle(45.0);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    // m_slamController.request_planning_joint_pose();
    // m_slamController.request_planning_joint_pose_to_sleep();

    ros::WallDuration(1.0).sleep();

    // m_slamController.request_execute_moveit_plan();

    // ros::WallDuration(1.0).sleep();

    // m_slamController.request_open_gripper();

    ros::WallDuration(1.0).sleep();

    m_slamController.request_pick_up();

    ros::waitForShutdown();

    return 0;
}