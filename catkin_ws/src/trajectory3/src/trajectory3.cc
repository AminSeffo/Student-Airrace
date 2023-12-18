#include <planner.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(20.0),
        max_a_(10.0),
        gates_(10),
        max_laps_(10),
        explored_(false),
        on_a_lap(false),
        laps(0),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) {

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 10);
    // subscriber for Odometry
    sub_odom_ = nh.subscribe("current_state_est", 1, &BasicPlanner::uavOdomCallback, this);
}
void BasicPlanner::exploreCallback(const std_msgs::Bool explored){

    std::cout << "exploreCallback" << explored.data << std::endl;
    if (explored.data == true){
        explored_ = true;
    }
}
// Callback to get cur0rent waypoints from navigation, receives a vector of 3d vectors
void BasicPlanner::waypointCallback() {
    ros::NodeHandle nh;
    Eigen::Vector3d goal_position, goal_velocity;
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
    if (waypoints_.empty()){
        std::cout << "waypoints_ is empty" << std::endl;
        for (int i = 0; i < gates_*4; i = i + 4){
            double middlex, middley, middlez;
            middlex = (current.out_est[i].x + current.out_est[i+2].x) / 2;
            middley = (current.out_est[i].y + current.out_est[i+2].y) / 2;
            middlez = (current.out_est[i].z + current.out_est[i+2].z) / 2;
            waypoints_.push_back(Eigen::Vector3d(middlex, middley, middlez));
        }
    }
    else {
        for (int i = 0; i < gates_*4; i = i + 4){
            double middlex, middley, middlez;
            middlex = (current.out_est[i].x + current.out_est[i+2].x) / 2;
            middley = (current.out_est[i].y + current.out_est[i+2].y) / 2;
            middlez = (current.out_est[i].z + current.out_est[i+2].z) / 2;
            waypoints_[i/4] = Eigen::Vector3d(middlex, middley, middlez);
        }
    }
    // get absolute value of the difference of waypoint[0] and current position
    double norm = (waypoints_[0] - current_pose_.translation()).norm();
    if (norm > 25 && explored_ == true && on_a_lap == true){
        on_a_lap = false;
    }
    // check if we are arriving at the first waypoint after a lap
    if (norm < 10 && explored_ == true && on_a_lap == false && laps != 0){
        laps = laps + 1;
        if (laps <= max_laps_){
            on_a_lap = true;
            std::cout << "exploreCallback" << " currently on lap " << laps << std::endl;
            for (int i = 0; i < waypoints_.size(); i++){
                std::cout << "waypoint " << i << ": " << waypoints_[i] << std::endl;
            }
            //std::cout << "no waypoint is (0, 0, 0)" << std::endl;
            mav_trajectory_generation::Trajectory trajectory;
            // set goal position to the last waypoint
            goal_position = Eigen::Vector3d(0, 0, 0);

            // set goal velocity to all 0.5 in x , y
            goal_velocity = Eigen::Vector3d(0, 0, 0);

            // remove last waypoint from list
            // waypoints_.pop_back();
            // plan trajectory
        
            planTrajectory(goal_position, goal_velocity, &trajectory);
            // publish trajectory
            publishTrajectory(trajectory);       
        }
    }
    // start trajectory after exploring
    else if (current_pose_.translation().norm() < 8 && explored_ == true && on_a_lap == false && laps == 0 && current_velocity_.norm() < 0.1){
        laps = laps + 1;
        if (laps <= max_laps_){
            on_a_lap = true;
            std::cout << "exploreCallback" << " currently on lap " << laps << std::endl;
            for (int i = 0; i < waypoints_.size(); i++){
                std::cout << "waypoint " << i << ": " << waypoints_[i] << std::endl;
            }
            //std::cout << "no waypoint is (0, 0, 0)" << std::endl;
            mav_trajectory_generation::Trajectory trajectory;
            // set goal position to the last waypoint
            goal_position = Eigen::Vector3d(0, 0, 0);

            // set goal velocity to all 0.5 in x , y
            goal_velocity = Eigen::Vector3d(0, 0, 0);

            // plan trajectory
            planTrajectory(goal_position, goal_velocity, &trajectory);
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
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
void BasicPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints_ and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 3 vertices:
    // Start = current position
    // Middle = desired position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    /******* Configure start point *******/
    Eigen::Vector3d start_waypoint;
    int st;
     /******* Configure start point *******/
    if (laps == 1){
        // if it is the first lap, start from the first waypoint_pos
        start_waypoint = Eigen::Vector3d(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]);
        st = 1;
    }
    else{
        // if it is not the first lap, start from the second waypoint_pos
        start_waypoint = Eigen::Vector3d(waypoints_[1][0], waypoints_[1][1], waypoints_[1][2]);
        st = 2;
    }
    start.makeStartOrEnd(start_waypoint,
                         derivative_to_optimize);

    // add waypoint to list
    vertices.push_back(start);
    int size;
    // use the waypoint list to create the middle waypoints_
    size = waypoints_.size();
    for (int i = st; i < size; i++) {
        // free constraints of your middle waypoint before reusing it
        middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);

        // set position
        Eigen::Vector3d position(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, position);
        vertices.push_back(middle);
        // add waypoint as the middle point of the current and next waypoint
        // if (i == size - 1){
        //     break;
        //     }
        // middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
        // middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
        // middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
        // Eigen::Vector3d middle_point((waypoints_[i][0] + waypoints_[i+1][0]) / 2,
        //                              (waypoints_[i][1] + waypoints_[i+1][1]) / 2,
        //                              (waypoints_[i][2] + waypoints_[i+1][2]) / 2);
        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, middle_point);
        // vertices.push_back(middle);
    }

    /******* Configure end point *******/
    if (laps != max_laps_) {
        // if it is not the last lap then set the end point as the first waypoint
        end.makeStartOrEnd(Eigen::Vector3d(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]),
                       derivative_to_optimize);
    }
    else {
        // if it is the last lap then set the end point as the desired position and velocity
        end.makeStartOrEnd(Eigen::Vector3d(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]),
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
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
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
    ros::init(argc, argv, "trajectory3");
    ros::NodeHandle nh_;
	
    //ros::Duration(1.0).sleep(); // sleep for 1 second
    BasicPlanner planner(nh_);  // instantiate basic planner
    ros::Subscriber explore_sub = nh_.subscribe("/explored", 1, &BasicPlanner::exploreCallback, &planner);
    // waypoint_sub_  = nh_.subscribe("waypoints", 1, &BasicPlanner::waypointCallback, &planner);
    // Spin forever
    // call waypointCallback 
    while (ros::ok()) {
        ros::spinOnce();
        planner.waypointCallback();
    }
    ros::spin();

}