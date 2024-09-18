#include <Explorer.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "hector_explorer");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    ROS_INFO("Explorer Node entered!");
    
    Quadrotor quad(std::ref(node_handle));
    quad.takeoff();
    return 0;
}