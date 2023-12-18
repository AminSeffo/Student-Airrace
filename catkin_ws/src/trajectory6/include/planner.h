#ifndef TRAJECTORY_PKG_PLANNER_H
#define TRAJECTORY_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <map_generation/CreateMap.h>
#include <std_msgs/Bool.h>


class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void setMaxSpeed(double max_v);

    void exploreCallback(const std_msgs::Bool explored);

    // declare waypointCallback that receives a vector of 3d vectors
    void waypointCallback();

    void planTrajectory(const Eigen::VectorXd& goal_pos,const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory);

    void planTrajectory6(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        const Eigen::VectorXd& start_vel,
                        double v_max, double a_max,
                        mav_trajectory_generation::Trajectory* trajectory, int flag);

    void publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
    
private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_waypoint_;
    ros::Subscriber explore_sub;
    ros::ServiceClient waypoint_sub_;
    ros::NodeHandle& nh_;
    // define current waypoint list each waypoint is an Eigen 3d vector at the start the list is empty
    // initialize an empty vector of waypoints
    std::vector<Eigen::Vector3d> waypoints_pos;
    std::vector<Eigen::Vector3d> waypoints_rot;
    std::vector<Eigen::Vector3d> waypoints_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;
    double gates_;
    int laps;
    int max_laps_;
    bool explored_;
    bool on_a_lap;

};
#endif // TRAJECTORY_PKG_PLANNER_H