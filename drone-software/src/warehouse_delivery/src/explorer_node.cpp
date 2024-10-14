#include <Explorer.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "delivery_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    ROS_INFO("Warehouse Delivery Node entered!");
    
    Quadrotor quad(std::ref(node_handle));

    ROS_INFO("Calling takeoff!");
    quad.takeoff();
    ROS_INFO("After takeoff!");

    ros::waitForShutdown();
    return 0;
}