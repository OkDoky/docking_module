/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 *  Editor : OkDoky(2023)
 */

#ifndef MPC_LOCAL_PLANNER_NODE_ROS_H
#define MPC_LOCAL_PLANNER_NODE_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/QR>
#include <iostream>
#include <math.h>
#include <fstream>

#include "ros/ros.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include "docking_tracker/nmpc_ros/mpc_planner.h"
#include "docking_tracker/nmpc_ros/trajectory.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>


using namespace std;

enum mpc_state{
    NOT_WORKING,
    GETPLAN,
    ROTATION,
    TRACKING,
    ONLY_POSITION_ARRIVED,
    ARRIVED 
};

std::string enumToString(mpc_state state){
    switch (state){
        case NOT_WORKING:
            return "NOT_WORKING";
        case GETPLAN:
            return "GETPLAN";
        case ROTATION:
            return "ROTATION";
        case TRACKING:
            return "TRACKING";
        case ONLY_POSITION_ARRIVED:
            return "ONLY_POSITION_ARRIVED";
        case ARRIVED:
            return "ARRIVED";
    }
}

namespace mpc_ros{

    class MPCPlannerROS
    {
        public:
            MPCPlannerROS();
            ~MPCPlannerROS();

            // for visualisation, publishers of global and local plan
            ros::Publisher g_plan_pub_, l_plan_pub_;
            ros::Publisher mpc_state_pub_;

            // Local planner plugin functions
            void initialize();
            void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> transformed_plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                            const std::vector<geometry_msgs::PoseStamped> transformed_plan);
            Trajectory findBestPath(geometry_msgs::Twist& drive_velocities,
                                    const std::vector<geometry_msgs::PoseStamped> transformed_plan);
            void runRotationMotion(geometry_msgs::Twist& cmd_vel);
            mpc_state getTrackingState();
            void setParam(); 
            void getLocalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan);

        private:      
            geometry_msgs::PoseStamped current_pose_;
            geometry_msgs::Twist _cmd_vel;
            // Flags
            bool initialized_;

        private:
        
            // Solve the model given an initial state and polynomial coefficients.
            // Return the first actuatotions.
            vector<double> mpc_x;
            vector<double> mpc_y;
            vector<double> mpc_theta;

            ros::NodeHandle _nh;
            ros::Subscriber _sub_odom, _sub_global_plan;
            ros::Publisher _pub_downsampled_path, _pub_mpctraj;
            ros::Publisher cmd_vel_pub_;
            ros::Publisher cte_pub_;
            ros::ServiceClient _client_set_start;
            tf2_ros::Buffer *tf_;  ///
            
            nav_msgs::Odometry _odom;
            nav_msgs::Path _odom_path, _mpc_traj, _g_path, _l_path;

            // init robot pose
            double _rx, _ry, _rtheta;
            double _fvx, _fvy, _fvw; // linear x vel, linear y vel, angular yaw vel

            // init goal tolerance
            double _xy_tolerance, _yaw_tolerance;

            double _default_max_linear_speed, _default_max_angular_speed;
            double _safety_speed;

            mpc_state _arrival_state;
            ros::Timer timer_;

            MPC _mpc;
            map<string, double> _mpc_params;
            double _control_frequency, _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
                _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value, _path_length,
                _max_linear_speed, _min_linear_speed;

            double _dt, _w, _throttle, _speed, _max_speed;
            double _pathLength, _goalRadius, _waypointsDist;
            int _downSampling;
            bool _debug_info, _delay_mode;
            double polyeval(Eigen::VectorXd coeffs, double x);
            Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
            std::string str_state_before;

            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
            void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
            void controlLoopCB(const ros::TimerEvent&);
    };
};
#endif /* MPC_LOCAL_PLANNER_NODE_ROS_H */
