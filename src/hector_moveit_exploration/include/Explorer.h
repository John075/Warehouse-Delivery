#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include "hector_moveit_exploration/MoveAction.h"
#include <octomap_msgs/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>
#include <gazebo_ros_link_attacher/Attach.h>

#include <octomap/OcTree.h>


#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#define XMIN -24.5
#define XMAX 25.0
#define YMIN -16
#define YMAX 25.0
#define ZMIN 0.2
#define ZMAX 25.0

#define EPSILON 1e-4

typedef std::pair<double,geometry_msgs::Pose> DistancedPoint;

class Compare{
    public: 
        bool operator()(DistancedPoint& lhs, DistancedPoint & rhs)
        {
            return lhs.first < rhs.first;
        }
};
typedef std::priority_queue<DistancedPoint,std::vector<DistancedPoint>, Compare> DistancedPointPriorityQueue;

#include <iostream>
#include <chrono>

using namespace std;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;

#include <omp.h>
#define PATCH_LIMIT 1
class Quadrotor{
    private:
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction> trajectory_client;
        std::unique_ptr<robot_state::RobotState> start_state;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        const double takeoff_altitude = 9;
        bool odom_received, trajectory_received;
        bool isPathValid;
        bool collision;

        geometry_msgs::Pose odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;

        ros::Subscriber base_sub,plan_sub,move_sub;
        ros::ServiceClient motor_enable_service; 
        ros::ServiceClient planning_scene_service;
        ros::ServiceClient attach_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;
    
        const std::string PLANNING_GROUP = "DroneBody";

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg);

        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

        void collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback);

        void moveCallback(const hector_moveit_exploration::MoveAction::ConstPtr& msg);

        bool go(geometry_msgs::Pose& target_);
    
    public:
        Quadrotor(ros::NodeHandle& nh);
        void takeoff();
        void run();
        
};