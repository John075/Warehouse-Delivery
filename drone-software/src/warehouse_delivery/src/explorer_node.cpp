#include <Explorer.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "warehouse_delivery");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    ROS_INFO("Warehouse Delivery Node entered!");
    
    Quadrotor quad(std::ref(node_handle));
    quad.takeoff();

    ros::waitForShutdown();
    return 0;
}