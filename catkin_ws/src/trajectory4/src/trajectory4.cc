#include <planner.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(20),
        max_a_(10.0),
        max_ang_v_(7.0),
        max_ang_a_(5.0),
        gates_(10),
        max_laps_(10),
        explored_(false),
        on_a_lap(false),
        laps(0),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_angular_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) {

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
    // subscriber for Odometry
    sub_odom_ = nh.subscribe("current_state_est", 1, &BasicPlanner::uavOdomCallback, this);   
}
void BasicPlanner::exploreCallback(const std_msgs::Bool explored){

    std::cout << "exploreCallback" << explored.data << std::endl;
    if (explored.data == true){
        explored_ = true;
    }
}
void BasicPlanner::waypointCallback() {
    ros::NodeHandle nh;
    Eigen::Vector4d goal_pos, goal_vel;
    // Create a service client to call the service map
    waypoint_sub_ = nh.serviceClient<map_generation::CreateMap>("map");
    // get response out_est from service map
    map_generation::CreateMap srv;
    waypoint_sub_.call(srv);
    // get response
    int size = gates_* 4 + 1;
    // call service map
    map_generation::CreateMap::Response current;
    current = srv.response;
    // check if waypoints is empty
    if (waypoints4d_.empty()){
        for (int i = 0; i < gates_*4; i = i + 4){
            double middlex, middley, middlez;
            middlex = (current.out_est[i].x + current.out_est[i+2].x) / 2;
            middley = (current.out_est[i].y + current.out_est[i+2].y) / 2;
            middlez = (current.out_est[i].z + current.out_est[i+2].z) / 2;
            double angle = atan2(current.out_est[i+2].y - current.out_est[i].y, current.out_est[i+2].x - current.out_est[i].x);
            angle += M_PI / 2.0;
            // make sure the angle is between 0 and 2pi
            while (angle < 0){
                angle += 2 * M_PI;
            }
            while (angle > 2 * M_PI){
                angle -= 2 * M_PI;
            }
            waypoints4d_.push_back(Eigen::Vector4d(middlex, middley, middlez, angle));
        }
    }
    else {
        for (int i = 0; i < gates_*4; i = i + 4){
            double middlex, middley, middlez;
            middlex = (current.out_est[i].x + current.out_est[i+2].x) / 2;
            middley = (current.out_est[i].y + current.out_est[i+2].y) / 2;
            middlez = (current.out_est[i].z + current.out_est[i+2].z) / 2;
            double angle = atan2(current.out_est[i+2].y - current.out_est[i].y, current.out_est[i+2].x - current.out_est[i].x);
            angle += M_PI / 2.0;
            // make sure the angle is between 0 and 2pi
            while (angle < 0){
                angle += 2 * M_PI;
            }
            while (angle > 2 * M_PI){
                angle -= 2 * M_PI;
            }
            waypoints4d_[i/4] = Eigen::Vector4d(middlex, middley, middlez, angle);
        }
    }

    Eigen::Vector3d waypoint_position = waypoints4d_[0].head<3>();
    double norm = (waypoint_position - current_pose_.translation()).norm();
    if (norm > 25 && explored_ == true && on_a_lap == true){
        on_a_lap = false;
    }
    if (norm < 10 && explored_ == true && on_a_lap == false && laps != 0){
        laps = laps + 1;
        if (laps <= max_laps_){
            on_a_lap = true;
            //std::cout << "exploreCallback" << " currently on lap " << laps << std::endl;
            for (int i = 0; i < waypoints4d_.size(); i++){
                std::cout << "waypoint " << i << ": " << waypoints4d_[i] << std::endl;
            }
            //std::cout << "no waypoint is (0, 0, 0)" << std::endl;
            mav_trajectory_generation::Trajectory trajectory;
            // set goal position to the last waypoint
            goal_pos= Eigen::Vector4d(0, 0, 0, 0);

            // set goal velocity to all 0
            goal_vel= Eigen::Vector4d(0, 0, 0, 0);

            // plan trajectory
            Eigen::Vector4d start_pos_4d, start_vel_4d;
            start_pos_4d << current_pose_.translation(), mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
            start_vel_4d << current_velocity_ , 0.0;
            planTrajectory4(
                goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
                &trajectory);
            // publish trajectory
            publishTrajectory(trajectory);       
        }
    }
    else if (current_pose_.translation().norm() < 8 && explored_ == true && on_a_lap == false && laps == 0 && current_velocity_.norm() < 0.1){
        laps = laps + 1;
        if (laps <= max_laps_){
            on_a_lap = true;
            //std::cout << "exploreCallback" << " currently on lap " << laps << std::endl;
            for (int i = 0; i < waypoints4d_.size(); i++){
                std::cout << "waypoint " << i << ": " << waypoints4d_[i] << std::endl;
            }
            //std::cout << "no waypoint is (0, 0, 0)" << std::endl;
            mav_trajectory_generation::Trajectory trajectory;
            // set goal position to the last waypoint
            goal_pos= Eigen::Vector4d(0, 0, 0, 0);

            // set goal velocity to all 0.5 in x , y
            goal_vel= Eigen::Vector4d(0, 0, 0, 0);

            // plan trajectory
            Eigen::Vector4d start_pos_4d, start_vel_4d;
            start_pos_4d << current_pose_.translation(), mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
            std::cout << "start_pos_4d: " << start_pos_4d[3] << std::endl;
             //       (Eigen::Quaterniond)current_pose_.rotation());
            start_vel_4d << current_velocity_ , 0.0;
            planTrajectory4(
                goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
                &trajectory);
            // publish trajectory
            publishTrajectory(trajectory);       
        }
    }
    
}
// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
    tf::vectorMsgToEigen(odom->twist.twist.angular, current_angular_velocity_);
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
void BasicPlanner::planTrajectory4(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel,
                                    double v_max, double a_max,
                                    mav_trajectory_generation::Trajectory* trajectory) {

    trajectory->clear();
    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 4;

    // Array for all waypoints4d_ and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 3 vertices:
    // Start = current position
    // Middle = desired position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    Eigen::Vector4d start_waypoint;
    int st;
     /******* Configure start point *******/
    if (laps == 1){
        // if it is the first lap, start from the first waypoint_pos
        start_waypoint = Eigen::Vector4d(waypoints4d_[0][0], waypoints4d_[0][1], waypoints4d_[0][2], waypoints4d_[0][3]);
        st = 1;
    }
    else{
        // if it is not the first lap, start from the second waypoint_pos
        start_waypoint = Eigen::Vector4d(waypoints4d_[1][0], waypoints4d_[1][1], waypoints4d_[1][2], waypoints4d_[1][3]);
        st = 2;
    }
    start.makeStartOrEnd(start_waypoint,
                         derivative_to_optimize);
    // add waypoint to list
    vertices.push_back(start);
    
    int size;
    // use the waypoint list to create the middle waypoints4d_
    size = waypoints4d_.size();
    for (int i = st; i < size; i++) {
        // free constraints of your middle waypoint before reusing it
        middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);

        // set position
        Eigen::Vector4d position(waypoints4d_[i][0], waypoints4d_[i][1], waypoints4d_[i][2], waypoints4d_[i][3]);
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, position);
        vertices.push_back(middle);
    }

    /******* Configure end point *******/
    if (laps != max_laps_) {
        // if it is not the last lap then set the end point as the first waypoint
        end.makeStartOrEnd(Eigen::Vector4d(waypoints4d_[0][0], waypoints4d_[0][1], waypoints4d_[0][2], waypoints4d_[0][3]),
                       derivative_to_optimize);
    }
    else {
        // if it is the last lap then set the end point as the desired position and velocity
        end.makeStartOrEnd(Eigen::Vector4d(waypoints4d_[0][0], waypoints4d_[0][1], waypoints4d_[0][2], waypoints4d_[0][3]),
                       derivative_to_optimize);
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            goal_vel);
    }

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);

    //return true;
}

void BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

   // return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh_;

    BasicPlanner planner(nh_);  // instantiate basic planner
    ros::Subscriber explore_sub = nh_.subscribe("/explored", 1, &BasicPlanner::exploreCallback, &planner);

    // call waypointCallback 
    while (ros::ok()) {
        ros::spinOnce();
        planner.waypointCallback();
    }
    ros::spin();

}