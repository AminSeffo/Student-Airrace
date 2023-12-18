//Autor: Amin Seffo
//Date: 28.02.2023

#include <ros/ros.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_listener.h>


#define PI M_PI

ros::Publisher takeoff_pub;

void calibrationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  static bool calibrated = false;
  static double start_yaw = 0.0;
  
  if (!calibrated)
  {
    // Get the yaw (z-axis rotation) from the quaternion in the messagedddd
    double q0 = msg->pose.pose.orientation.w;
    double q1 = msg->pose.pose.orientation.x;
    double q2 = msg->pose.pose.orientation.y;
    double q3 = msg->pose.pose.orientation.z;
    double yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

    // Save the starting yaw for calibration
    start_yaw = yaw;
    calibrated = true;

    // Publish the calibration as a waypoint
    trajectory_msgs::MultiDOFJointTrajectory waypoint;
    // Quantities to fill in
    tf::Transform desired_pose(tf::Transform::getIdentity());
    tf::Vector3 origin(0,0,0);
    geometry_msgs::Twist velocity;
    velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
    velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
    geometry_msgs::Twist acceleration;
    acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
    acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;
    tf::Vector3 displacement(0,0,5);
    desired_pose.setOrigin(origin+displacement);
    tf::Quaternion q;
    if (start_yaw !=yaw)
    {
      ROS_WARN("Calibration is needed!");
      ROS_INFO("Calibration is in progress....");

      q.setRPY(0,0,yaw);
      desired_pose.setRotation(q);
      trajectory_msgs::MultiDOFJointTrajectory msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";
      msg.points.resize(1);
      msg.points[0].transforms.resize(1);
      msg.points[0].transforms[0].translation.x = desired_pose.getOrigin().x();
      msg.points[0].transforms[0].translation.y = desired_pose.getOrigin().y();
      msg.points[0].transforms[0].translation.z = desired_pose.getOrigin().z();
      msg.points[0].transforms[0].rotation.x = desired_pose.getRotation().x();
      msg.points[0].transforms[0].rotation.y = desired_pose.getRotation().y();
      msg.points[0].transforms[0].rotation.z = desired_pose.getRotation().z();
      msg.points[0].transforms[0].rotation.w = desired_pose.getRotation().w();
      msg.points[0].velocities.resize(1);
      msg.points[0].velocities[0] = velocity;
      msg.points[0].accelerations.resize(1);
      msg.points[0].accelerations[0] = acceleration;
      //wait for  seconds
      ros::Duration(0.1).sleep();
      takeoff_pub.publish(msg);
      ROS_WARN("Calibration is done.");
      ROS_WARN("Exploring is started. Good luck!");

    }
    else
    {
     ROS_WARN("Calibration is not needed!");
     ROS_WARN("Exploring is started. Good luck!");

    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_pub");
  ros::NodeHandle n;
  takeoff_pub= n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 10);
  ros::Subscriber odom_sub = n.subscribe("/current_state_est", 10,calibrationCallback);
  ros::spin();

  return 0;
}