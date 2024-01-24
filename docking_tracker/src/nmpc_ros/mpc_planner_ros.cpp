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
 */

#include "docking_tracker/nmpc_ros/mpc_planner_ros.h"

using namespace std;
using namespace Eigen;

namespace mpc_ros{

	MPCPlannerROS::MPCPlannerROS()
    {
        // initialize planner
        initialize();
    }
	MPCPlannerROS::~MPCPlannerROS() {}

	void MPCPlannerROS::initialize(){

        ros::NodeHandle private_nh("~/");
        ROS_WARN("[ROSMPC] start to initialize MPCPlannerROS");
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        cmd_vel_pub_ = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        mpc_state_pub_ = _nh.advertise<std_msgs::String>("docking/status", 1);
        cte_pub_ = _nh.advertise<std_msgs::Float64>("mpc_cte", 1);
        debug_first_local_plan_point_ = _nh.advertise<geometry_msgs::PoseStamped>("debug_first_local_plan_point", 1);

        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        std::string controller_frequency_param_name;
        double controller_frequency = 10.0;
        
        _dt = double(1.0/controller_frequency); // time step duration dt in s 

        //Publishers and Subscribers
        _sub_odom   = _nh.subscribe("odom", 1, &MPCPlannerROS::odomCB, this);
        _sub_global_plan = _nh.subscribe("/plan", 1, &MPCPlannerROS::pathCB, this);
        _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);// MPC trajectory output
        _pub_downsampled_path  = _nh.advertise<nav_msgs::Path>("mpc_reference", 1); // reference path for MPC ///mpc_reference 
        _client_set_marker_detect = _nh.serviceClient<std_srvs::SetBool>("set_marker_detect");
        _server_set_docking = _nh.advertiseService("set_docking", &MPCPlannerROS::dockingCB, this);

        // init robot state
        _rx = 0.0;
        _ry = 0.0;
        _rtheta = 0.0;
        _fvx = 0.0;
        _fvy = 0.0;
        _fvw = 0.0;

        _crossproduct_prev = 0.0;

        // init mpc params
        ROS_WARN("[ROSMPC] start to initialize params.");
        private_nh.param<double>("control_frequency", _control_frequency, 10.0);
        private_nh.param<double>("mpc_steps", _mpc_steps, 20.0);
        private_nh.param<double>("ref_cte", _ref_cte, 0.0);
        private_nh.param<double>("ref_etheta",_ref_etheta, 0.0);
        private_nh.param<double>("ref_vel",_ref_vel, 0.6);
        private_nh.param<double>("w_cte",_w_cte, 2000.0);
        private_nh.param<double>("w_etheta",_w_etheta, 500.0);
        private_nh.param<double>("w_vel",_w_vel, 1000.0);
        private_nh.param<double>("w_angvel",_w_angvel, 0.0);
        private_nh.param<double>("w_accel",_w_accel, 10.0);
        private_nh.param<double>("w_angvel_d",_w_angvel_d, 10.0);
        private_nh.param<double>("w_accel_d",_w_accel_d, 10.0);
        private_nh.param<double>("max_angvel",_max_angvel, 0.6);
        private_nh.param<double>("max_throttle",_max_throttle, 1.0);
        private_nh.param<double>("bound_value",_bound_value, 1.0e3);
        private_nh.param<double>("path_length", _path_length, 5.0);
        private_nh.param<double>("max_linear_speed", _max_linear_speed, 0.7);
        private_nh.param<double>("min_linear_speed", _min_linear_speed, -0.3);
        private_nh.param<double>("xy_tolerance", _xy_tolerance, 0.05);
        private_nh.param<double>("yaw_tolerance", _yaw_tolerance, 0.05);
        private_nh.param<bool>("heading_front", _heading_front, true);

        _dt = double(1.0/_control_frequency);
        _default_max_linear_speed = _max_linear_speed;
        _default_max_angular_speed = _max_angvel;
        _safety_speed = 0.02;
        _requested_cancel = false;

        setParam();

        //Init variables
        _throttle = 0.0; 
        _w = 0.0;
        _speed = 0.0;
        _arrival_state = NOT_WORKING;
        str_state_before = "None";
        initialized_ = true;
        double marker_displacement, path_displacement;
        _path_displacement = 0.1;
        _nh.param<double>("path_generate/marker_displacement", marker_displacement, 0.05);
        _nh.param<double>("path_generate/path_displacement", path_displacement, 0.5);
        _path_displacement = _path_displacement + path_displacement + marker_displacement;
        ROS_WARN("[ROSMPC] start to initialize timer event.");
        timer_ = _nh.createTimer(ros::Duration(_dt), &MPCPlannerROS::controlLoopCB, this);
        ROS_WARN("[ROSMPC] finish initialize timer control loop.");

    }

	void MPCPlannerROS::setParam(){
        //Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        _mpc_params["STEPS"]    = _mpc_steps;
        _mpc_params["REF_CTE"]  = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"]    = _ref_vel;
        _mpc_params["W_CTE"]    = _w_cte;
        _mpc_params["W_EPSI"]   = _w_etheta;
        _mpc_params["W_V"]      = _w_vel;
        _mpc_params["W_ANGVEL"]  = _w_angvel;
        _mpc_params["W_A"]      = _w_accel;
        _mpc_params["W_DANGVEL"] = _w_angvel_d;
        _mpc_params["W_DA"]     = _w_accel_d;
        _mpc_params["ANGVEL"]   = _max_angvel;
        _mpc_params["MAXTHR"]   = _max_throttle;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);      
    }

    void MPCPlannerROS::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> transformed_plan){
        nav_msgs::Path odom_frame_path;
        odom_frame_path.header.frame_id = "odom";
        odom_frame_path.header.stamp = ros::Time::now();
        odom_frame_path.poses = transformed_plan;
        l_plan_pub_.publish(odom_frame_path);
    }

	bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        getLocalPlan(transformed_plan);

        publishLocalPlan(transformed_plan);
        if (_arrival_state == ARRIVED){
            return true;
        }
        else if (_arrival_state == ONLY_POSITION_ARRIVED){
            // rotateMotion();
            return true;
        }
        else if (_arrival_state == TRACKING){
            bool isOk = mpcComputeVelocityCommands(cmd_vel, transformed_plan);
            if (!isOk)
                ROS_WARN("[MPCROS] failed to produce path.");
            return isOk;
        }
        return false;
    }

    // Timer: Control Loop (closed loop nonlinear MPC)
    bool MPCPlannerROS::mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                                   const std::vector<geometry_msgs::PoseStamped> transformed_plan)
    {

        //compute what trajectory to drive along
        geometry_msgs::Twist drive_cmds;

        // call with updated footprint
        Trajectory path = findBestPath(drive_cmds, transformed_plan);

        //pass along drive commands
        cmd_vel = drive_cmds;

        //if we cannot move... tell someone
        if(path.cost_ < 0) {
            ROS_WARN("[MPCROS] cannot find best Trajectory..");
            return false;
        }

        ROS_DEBUG_NAMED("MPCROS", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        return true;
    }

    Trajectory MPCPlannerROS::findBestPath(geometry_msgs::Twist& drive_velocities,
                                           const std::vector<geometry_msgs::PoseStamped> transformed_plan){

        Trajectory result_traj_;
        geometry_msgs::PoseStamped goal_pose = _g_path.poses.back();
        result_traj_.cost_ = 1;

        /*
        *
        *  MPC Control Loop
        * 
        */
        // Update system states: X=[x, y, theta, v]
        const double px = _rx; //pose: odom frame
        const double py = _ry;
        double theta = _rtheta;
        const double v = hypot(_fvx,_fvy); //twist: body fixed frame
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;

        //Update path waypoints (conversion to odom frame)
        nav_msgs::Path odom_path = nav_msgs::Path();
        double total_length = 0.0;
        int sampling = _downSampling;

        //find waypoints distance
        if(_waypointsDist <=0.0)
        {        
            double dx = transformed_plan[1].pose.position.x - transformed_plan[0].pose.position.x;
            double dy = transformed_plan[1].pose.position.y - transformed_plan[0].pose.position.y;
            _waypointsDist = sqrt(dx*dx + dy*dy);
            _downSampling = 2;
        }
        if (transformed_plan.size() > _downSampling*2){
            // Cut and downsampling the path
            for(int i =0; i< transformed_plan.size(); i++)
            {
                if(total_length > _path_length)
                    break;

                if(sampling == _downSampling)
                {
                    odom_path.poses.push_back(transformed_plan[i]);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }
        } else{
            odom_path.poses = transformed_plan;
        }
        
        if(odom_path.poses.size() > 1)
        {
            // publish odom path
            odom_path.header.frame_id = "odom";
            odom_path.header.stamp = ros::Time::now();
            _pub_downsampled_path.publish(odom_path);
        }
        else
        {
            ROS_WARN("[MPCROS] Failed to path generation since small down-sampling path.");
            _waypointsDist = -1;
            result_traj_.cost_ = -1;
            return result_traj_;
        }

        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);
        
        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
        }

        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 
        const double cte  = polyeval(coeffs, 0.0);
        double etheta = atan(coeffs[1]);
        std_msgs::Float64 cte_msg;
        cte_msg.data = cte;
        cte_pub_.publish(cte_msg);
        // ROS_WARN("[ROSNMPC] cte : %.4f", cte);
        // ROS_WARN("[ROSNMPC] etheta : %.4f", etheta);

        // Global coordinate system about theta
        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        if (N_sample < 1) N_sample = 1; 
        gx = odom_path.poses[N_sample].pose.position.x - odom_path.poses[0].pose.position.x;
        gy = odom_path.poses[N_sample].pose.position.y - odom_path.poses[0].pose.position.y;
        

        double temp_theta = theta;
        double traj_deg = atan2(gy,gx); // odom frame
        double PI = 3.141592;

        // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
        if(temp_theta <= -PI + traj_deg) 
            temp_theta = temp_theta + 2 * PI;
        
        // Implementation about theta error more precisly
        if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
            etheta = temp_theta - traj_deg;
        else
            etheta = 0;  

        // Difference bewteen current position and goal position
        const double x_err = goal_pose.pose.position.x -  _rx;
        const double y_err = goal_pose.pose.position.y -  _ry;
        const double goal_err = hypot(x_err, y_err);

        VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt), body frame
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; //v = v + a * dt
            
            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }

        // Solve MPC Problem
        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        ros::Time end = ros::Time::now();
            
        // MPC result (all described in car frame), output = (acceleration, w)        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration m/sec^2

        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_linear_speed)
            _speed = _max_linear_speed;
        if(_speed <= _min_linear_speed)
            _speed = _min_linear_speed;

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = "base_footprint"; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;

        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];

            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _mpc_traj.poses.push_back(tempPose); 
        }     

        if(result_traj_.cost_ >= 0){
            drive_velocities.linear.x = _speed;
            drive_velocities.angular.z = _w;
        }
        
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);
        return result_traj_;
    }

    void MPCPlannerROS::runRotationMotion(geometry_msgs::Twist& cmd_vel){
        double _etheta;
        int _index = 0;
        if (_arrival_state == ONLY_POSITION_ARRIVED)
            int _index = _l_path.poses.size() - 1;
        _etheta = tf::getYaw(_l_path.poses[_index].pose.orientation) - _rtheta;
        cmd_vel.angular.z = _etheta + _yaw_tolerance*_etheta/abs(_etheta);
    }

    double MPCPlannerROS::crossProductOnTheLine(const std::vector<double>& point, const std::vector<double>& lineStart, double lineDirectionRadian){
        double lineDirectionX = cos(lineDirectionRadian);
        double lineDirectionY = sin(lineDirectionRadian);

        std::vector<double> pointVector = {point[0] - lineStart[0], point[1] - lineStart[1]};

        double crossProduct = lineDirectionX * pointVector[1] - lineDirectionY * pointVector[0];
        return crossProduct;
    }

    bool MPCPlannerROS::determinCrossTheLine(double crossproduct_prev, double crossproduct){
        bool result;
        if(crossproduct_prev == 0.0){
            result = false;
        }
        else{
            result = (crossproduct_prev * crossproduct < 0);
        }
        // ROS_WARN("[ROSNMPC] crossproduct_prev : %f, crossproduct_curr : %f || result : %f", crossproduct_prev, crossproduct, crossproduct_prev*crossproduct);
        return result;
    }

	void MPCPlannerROS::getTrackingState(){
        mpc_state reached_state = _arrival_state;
        
        if (_requested_cancel){
            reached_state = CANCEL_REQUESTED;
            _requested_cancel = false;
            setState(reached_state);
            return;
        }
        int last_index = _l_path.poses.size() - 1;
        double _dx = _l_path.poses[last_index].pose.position.x - _rx;
        double _dy = _l_path.poses[last_index].pose.position.y - _ry;
        double _goalyaw = tf::getYaw(_l_path.poses[last_index].pose.orientation);
        double _pdist = hypot(_dx, _dy);
        double _etheta = _goalyaw - _rtheta;
        double _heading_error = tf::getYaw(_l_path.poses[0].pose.orientation) - _rtheta;
        std::vector<double> _rpos = {_rx, _ry};
        std::vector<double> _goalpos = {_l_path.poses[last_index].pose.position.x, _l_path.poses[last_index].pose.position.y};

        double _crossproduct = crossProductOnTheLine(_rpos, _goalpos, _goalyaw + M_PI/2.0);
        bool _crossFlag = determinCrossTheLine(_crossproduct_prev, _crossproduct);
        _crossproduct_prev = _crossproduct;
        

        switch(reached_state){
            case GETPLAN:
                if (abs(_heading_error) > _yaw_tolerance){
                    reached_state = ROTATION;
                    setState(reached_state);
                }
                else {
                    reached_state = TRACKING;
                    setState(reached_state);
                }
                return;
            case ROTATION:
                if (abs(_heading_error) < _yaw_tolerance){
                    reached_state = TRACKING;
                    setState(reached_state);
                }
                return;
            case TRACKING:
                if (_pdist < _xy_tolerance && abs(_etheta) < _yaw_tolerance){
                    reached_state = ARRIVED;
                    setState(reached_state);
                }
                else if (_pdist < _xy_tolerance){
                    reached_state = ONLY_POSITION_ARRIVED;
                    setState(reached_state);
                }
                else if (_crossFlag) {
                    reached_state = NOT_WORKING;
                    setState(reached_state);
                    _crossproduct_prev = 0.0;
                    ROS_ERROR("[ROSNMPC] robot cross the errorline without reaching the goal");
                }
                // deceleration
                if (_pdist < (_max_linear_speed + _safety_speed)*1/_max_throttle){
                    _max_linear_speed = _max_throttle * _pdist + _safety_speed;
                    _mpc_params["REF_VEL"]  = _max_linear_speed;
                    _mpc.LoadParams(_mpc_params);
                }
                return;
            case ONLY_POSITION_ARRIVED:
                if (abs(_etheta) < _yaw_tolerance){
                    reached_state = ARRIVED;
                    setState(reached_state);
                }
                return;
            case ARRIVED:
                reached_state = NOT_WORKING;
                setState(reached_state);
                return;
            case CANCEL_REQUESTED:
                reached_state = NOT_WORKING;
                setState(reached_state);
                return;
        }
    }

    void MPCPlannerROS::setState(const mpc_state& next_state){
        ROS_WARN("[ROSNMPC] state Transition %s -> %s", enumToString(_arrival_state).c_str(), enumToString(next_state).c_str());
        _arrival_state = next_state;
    }

    mpc_state MPCPlannerROS::getState(){
        return _arrival_state;
    }

    // Evaluate a polynomial.
    double MPCPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd MPCPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++) 
        {
            for (int i = 0; i < order; i++) 
                A(j, i + 1) = A(j, i) * xvals(j);
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    // CallBack: Update odometry
    void MPCPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        _odom = *odomMsg;
        _rx = _odom.pose.pose.position.x;
        _ry = _odom.pose.pose.position.y;
        _rtheta = tf::getYaw(_odom.pose.pose.orientation);
        _fvx = _odom.twist.twist.linear.x;
        _fvy = _odom.twist.twist.linear.y;
        _fvw = _odom.twist.twist.angular.z;
    }

    void MPCPlannerROS::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
    {   
        _g_path = *pathMsg;
        int last_index = _g_path.poses.size() - 1;
        double _dx = _g_path.poses[last_index].pose.position.x - _rx;
        double _dy = _g_path.poses[last_index].pose.position.y - _ry;
        double _dist = hypot(_dx, _dy);
        if (_arrival_state == WAITING_FOR_DETECTION){
            ROS_WARN("[ROSNMPC] get new plan");
            _l_path = *pathMsg;
            _arrival_state = GETPLAN;
            ROS_WARN("[ROSNMPC] state Transition WAITING_FOR_DETECTION -> GETPLAN");
        } else if (_arrival_state == TRACKING){
            ROS_DEBUG("[ROSNMPC] update global plan");
            _l_path = *pathMsg;
        }
    }

    bool  MPCPlannerROS::dockingCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
        if (req.data){
            if (_arrival_state == NOT_WORKING){
                _arrival_state = WAITING_FOR_DETECTION;
                std_srvs::SetBool _client;
                _client.request.data = true;
                _client_set_marker_detect.call(_client);
            }
        }else{
            if (_arrival_state != ARRIVED && _arrival_state != NOT_WORKING){
                _requested_cancel = true;
                std_srvs::SetBool _client;
                _client.request.data = false;
                _client_set_marker_detect.call(_client);
            }
        }
        res.success = true;
        return true;
    }

    void MPCPlannerROS::getLocalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan)
    {
        if  (_l_path.poses.empty()) return;
        nav_msgs::Path lPath = _l_path;
        double rx = _odom.pose.pose.position.x;
        double ry = _odom.pose.pose.position.y;
        
        double max_pow = 10e5;
        int iter_num = 0;
        for (int i = 0; i < lPath.poses.size(); i++)
        {
            double _dx = rx - lPath.poses[i].pose.position.x;
            double _dy = ry - lPath.poses[i].pose.position.y;
            double _pow = _dx*_dx + _dy*_dy;

            if (max_pow > _pow)
            {
                max_pow = _pow;
                iter_num = i;
                continue;
            }
            break;
        }
        for (int j=iter_num; j<lPath.poses.size(); j++)
        {
            transformed_plan.push_back(lPath.poses[j]);
        }
        _l_path.poses = transformed_plan;
    }

    void MPCPlannerROS::controlLoopCB(const ros::TimerEvent&)
    {
        std_msgs::String str_state;
        str_state.data = enumToString(_arrival_state);
        if (str_state.data != str_state_before){
            mpc_state_pub_.publish(str_state);
            str_state_before = str_state.data;
        }    

        geometry_msgs::Twist command_vel;
        if (_arrival_state == NOT_WORKING || _arrival_state == WAITING_FOR_DETECTION) {
            return;
        }
        getTrackingState();
        _arrival_state = getState();
        
        switch (_arrival_state){
            case ROTATION:
                runRotationMotion(command_vel);
                cmd_vel_pub_.publish(command_vel);
                debug_first_local_plan_point_.publish(_l_path.poses[0]);
                break;
            case TRACKING:{
                bool computeDone = computeVelocityCommands(command_vel);
                if (computeDone){
                    cmd_vel_pub_.publish(command_vel);
                }
                debug_first_local_plan_point_.publish(_l_path.poses[0]);
                break;
            }
            case ONLY_POSITION_ARRIVED:
                runRotationMotion(command_vel);
                cmd_vel_pub_.publish(command_vel);
                debug_first_local_plan_point_.publish(_l_path.poses[0]);
                break;
            case ARRIVED:{
                _max_linear_speed = _default_max_linear_speed;
                _max_angvel = _default_max_angular_speed;
                std_srvs::SetBool _client;
                _client.request.data = false;
                _client_set_marker_detect.call(_client);
                cmd_vel_pub_.publish(command_vel);
                debug_first_local_plan_point_.publish(_l_path.poses[0]);
                break;
            }
            case CANCEL_REQUESTED:
                ROS_WARN("[ROSNMPC] got cancel and publish stop");
                cmd_vel_pub_.publish(command_vel);
                break;

            default:
                break;
        }
    }
}
