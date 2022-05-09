#include "interbotix_simulation_controller/simulation_controller_obj.h"

SimulationController::SimulationController(ros::NodeHandle *node_handle):node(*node_handle)
{
    // camera publisher
    camera_pub = node.advertise< std_msgs::Float64>("/locobot/tilt_controller/command", 10);

    // the publisher to publish the goal of navigation
    nav_goal_pub = node.advertise<geometry_msgs::PoseStamped>("/locobot/move_base_simple/goal",10);
    
    // service
    srv_moveit_plan = node.advertiseService("moveit_plan",&SimulationController::Command_Process,this);

    static const std::string ARM_PLANNING_GROUP = "interbotix_arm";

    static const std::string GRIPPER_PLANNING_GROUP = "interbotix_gripper";

    arm_move_group = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
    arm_joint_model_group = arm_move_group->getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);

    gripper_move_group = new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP);
    gripper_joint_model_group = gripper_move_group->getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();    

    ROS_INFO_NAMED("simulation_controller","END effector link:%s",arm_move_group->getEndEffectorLink().c_str());
}

/// @brief Destructor for the simulation controller
SimulationController::~SimulationController()
{
    delete arm_move_group;
    delete arm_joint_model_group;

    delete gripper_move_group;
    delete gripper_joint_model_group;
}

/// @brief control the title angle
bool SimulationController::publish_camera_angle(const std_msgs::Float64 angle_data)
{
    std_msgs::Float64 angle;
    float angle_value = angle_data.data;
    angle.data = angle_value*M_PI/180.0f;

    camera_pub.publish(angle);
    return true;
}

/// @brief planning the position of end effector
bool SimulationController::plan_end_effector_pose(const geometry_msgs::Pose pose)
{
    arm_move_group->setPoseTarget(pose);
    bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("success:%d\n",success);
    return success;
}

/// @brief planning the position of joint
bool SimulationController::plan_joint_positions(const std::vector<double> joint_group_positions)
{
//   visual_tools->deleteAllMarkers();
  arm_move_group->setJointValueTarget(joint_group_positions);
  bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   visual_tools->publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   visual_tools->publishTrajectoryLine(saved_plan.trajectory_, arm_joint_model_group);
//   visual_tools->trigger();
  return success;
}

/// @brief planning the position of joint to home state
bool SimulationController::plan_joint_pose_to_home(void)
{
    std::map<std::string, double> home_joint_pose;
    home_joint_pose["elbow"] = 0.0;
    home_joint_pose["forearm_roll"] = 0.0;
    home_joint_pose["shoulder"] = 0.0;
    home_joint_pose["waist"] = 0.0;
    home_joint_pose["wrist_angle"] = 0.0;
    home_joint_pose["wrist_rotate"] = 0.0;

    arm_move_group->setJointValueTarget(home_joint_pose);
    bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

/// @brief planning the position of joint to sleep state
bool SimulationController::plan_joint_pose_to_sleep(void)
{
    std::map<std::string, double> sleep_joint_pose;
    sleep_joint_pose["elbow"] = 1.55;
    sleep_joint_pose["forearm_roll"] = 0.0;
    sleep_joint_pose["shoulder"] = -1.1;
    sleep_joint_pose["waist"] = 0.0;
    sleep_joint_pose["wrist_angle"] = 0.5;
    sleep_joint_pose["wrist_rotate"] = 0.0;

    arm_move_group->setJointValueTarget(sleep_joint_pose);
    bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

/// @brief open the gripper
bool SimulationController::open_gripper(void)
{
    std::map<std::string, double> open_gripper_pose;
    open_gripper_pose["left_finger"] = 0.037;
    open_gripper_pose["right_finger"] = -0.037;

    gripper_move_group->setJointValueTarget(open_gripper_pose);

    // bool success = (gripper_move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success = (gripper_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    success = (gripper_move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

/// @brief open the gripper
bool SimulationController::close_gripper(void)
{
    std::map<std::string, double> close_gripper_pose;
    close_gripper_pose["left_finger"] = 0.037;
    close_gripper_pose["right_finger"] = -0.037;

    gripper_move_group->setJointValueTarget(close_gripper_pose);

    // bool success = (gripper_move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success = (gripper_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    success = (gripper_move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

/// @brief Return the current end-effector pose relative to the 'world' frame
geometry_msgs::Pose SimulationController::moveit_get_end_effector_pose(void)
{
  return arm_move_group->getCurrentPose().pose;
}

/// @brief Execute a Moveit plan on the robot arm
bool SimulationController::execute_plan(void)
{
    bool success = (arm_move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}

void SimulationController::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger";
  posture.joint_names[1] = "right_finger";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.037;
  posture.points[0].positions[1] = -0.037;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void SimulationController::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger";
  posture.joint_names[1] = "right_finger";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.027;
  posture.points[0].positions[1] = -0.027;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

/// @brief pick up
bool SimulationController::pick_up(void)
{
    // arm_move_group->setPlanningTime(45.0);

    add_tennis_ball_as_object();

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "locobot/arm_base_link";
    tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, 0);
    orientation.setRPY(0, 0.4, 0);

    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.546;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = -0.075;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "locobot/arm_base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1;
    grasps[0].pre_grasp_approach.min_distance = 0.115;
    grasps[0].pre_grasp_approach.desired_distance = 0.135;

    // // Setting post-grasp retreat
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "locobot/arm_base_link";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closedGripper(grasps[0].grasp_posture);

    arm_move_group->pick("tennis_ball", grasps);

    return true;
}

/// @brief add tennis ball as object
void SimulationController::add_tennis_ball_as_object(void)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].header.frame_id = "locobot/arm_base_link";
    collision_objects[0].id = "tennis_ball";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.2;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.6;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.033;   // sin(theta/2) theta = 0
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;    // cos(theta/2)

    collision_objects[0].operation = collision_objects[2].ADD;

    planning_scene_interface->applyCollisionObjects(collision_objects);
}

// std_msgs::Float64 angle;
//     float angle_value = angle_data.data;
//     angle.data = angle_value*M_PI/180.0f;

//     camera_pub.publish(angle);
//     return true;

/// @brief publish the goal to navigation
bool SimulationController::publish_navigation_goal(const geometry_msgs::PoseStamped goal_pose)
{
    geometry_msgs::PoseStamped point;
    // point.point = 
    ROS_INFO("goal position x:%f",goal_pose.pose.position.x);

    nav_goal_pub.publish(goal_pose);

    return true;
}

bool SimulationController::Command_Process(interbotix_simulation_controller::MoveItPlan::Request &req, interbotix_simulation_controller::MoveItPlan::Response &res)
{
    bool success = false;
    std::string service_type;
    if(req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_CONTROL_CAMERA_ANGLE)
    {
        success = publish_camera_angle(req.camera_angle);
        service_type = "Adjust Camera Angle";
    }
    else if(req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_PLAN_POSE)
    {
        success = plan_end_effector_pose(req.ee_pose);
        service_type = "Planning EE pose";
    }
    else if(req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_PLAN_JOINT_POSE)
    {
        // float temp[] = req.joint_vector;
        // convert array from service to vector
        std::vector<double> joint_vector(std::begin(req.joint_vector),std::end(req.joint_vector));
        success = plan_joint_positions(joint_vector);
        service_type = "Planning Joint Pose";
    }
    else if (req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_EXECUTE)
    {
        success = execute_plan();
        service_type = "Execution";
    }
    else if (req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_HOME)
    {
        success = plan_joint_pose_to_home();
        service_type = "Plan to home";
    }
    else if (req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_SLEEP)
    {
        success = plan_joint_pose_to_sleep();
        service_type = "Plan to Sleep";
    }
    else if (req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_OPEN_GRIPPER)
    {
        success = open_gripper();
        service_type = "Open the gripper";
    }
    else if (req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_PICK_UP)
    {
        success = pick_up();
        service_type = "Pick up tennis ball";
    }
    else if(req.cmd == interbotix_simulation_controller::MoveItPlan::Request::CMD_NAVIGATION)
    {
        geometry_msgs::PoseStamped goal_pose = req.nav_goal_pose;
        success = publish_navigation_goal(goal_pose);
        service_type = "Publish the goal of navigation";
    }
    
    res.success = success;
    if (success)
        res.msg.data = service_type +" was successful!";
    else
        res.msg.data = service_type + " was not successful.";
    
    return true;
}