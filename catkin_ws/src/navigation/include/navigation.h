//Autor: Amin Seffo
//Date: 04.03.2023
//Last modified: 21.03.2023

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <gate.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

class Navigation
{
    private:
        ros::NodeHandle nh;
        ros::Publisher waypoint_pub;
        ros::Publisher explore_pub;
        ros::Subscriber aruco_marker_sub;
        ros::Subscriber dronePoseEstimate_sub;
        std::vector<Gate> gates;
        bool searchStrategyOneMarkerActive = false;
        int counterSearchSteps = 0;
        int timeout=0;
        int notDetectedTimeout=0;
        int reachedTimeout=0;
        float setpointReachedThreshold;
        bool finished = false;
        tf::Vector3 lastPositionSetpointWorld;
        tf::Quaternion lastOrientationSetpointWorld;
        tf::Vector3 currentDronePositionWorld;


    public:
        Navigation();
        int getNextGateId();
        //push the gate to the list
        void addGate(Gate gate);
        int getLeftArucoId(int gateId);
        int getRightArucoId(int gateId);
        bool nextGateIsVisible(int leftId, int rightId, const aruco_msgs::MarkerArray::ConstPtr& msg);
        bool nextLeftArucoIsVisible(int leftId, const aruco_msgs::MarkerArray::ConstPtr& msg);
        bool nextRightArucoIsVisible(int rightId, const aruco_msgs::MarkerArray::ConstPtr& msg);
        void sendToController(tf::Vector3 target, tf::Quaternion orientation);
        void arucoCallback(const aruco_msgs::MarkerArray::ConstPtr& msg);
        void dronePoseEstimateCallback(const nav_msgs::Odometry &msg);
        void convertToWorld(tf::Vector3 &worldPosition, tf::Quaternion &worldOrientation,
                                 tf::Vector3 &position, tf::Quaternion &orientation,
                                 tf::Vector3 &newWorldPosition, tf::Quaternion &newWorldOrientation);
        void stepAround(const aruco_msgs::MarkerArray::ConstPtr& msg, int Id);
        void stepForward();
};

#endif // NAVIGATION_H
