#include "docking_tracker/nmpc_ros/mpc_planner_ros.h" // Include the header file for the MPCPlannerROS class

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_planner_node");
    ros::NodeHandle nh;

    // Create an instance of the MPCPlannerROS class
    mpc_ros::MPCPlannerROS planner;

    // Spin the node to process callbacks
    ros::spin();

    return 0;
}