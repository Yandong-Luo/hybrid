#include "interbotix_simulation_controller/simulation_controller_obj.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"simulation_controller");

    /// @brief determine gazebo was paused or not
    bool paused_state = true;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;
    // Create instance of MoveIt interface
    SimulationController m_controller(&nh);
    ros::waitForShutdown();
    return 0;
}