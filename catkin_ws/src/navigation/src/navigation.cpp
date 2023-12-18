//Autor: Amin Seffo
//Date: 05.03.2023
//last update: 18.03.2023

#include "navigation.h"
#include <iostream>
#include <string>
#include <fstream>
#include <std_msgs/Bool.h>


Navigation::Navigation()
{   
    //Set up a publisher to send waypoint messages
    waypoint_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory",10);
    //Set up a subscriber to receive transform messages from aruco_node
    aruco_marker_sub = nh.subscribe("/aruco_marker_publisher/markers", 10, &Navigation::arucoCallback, this);
    dronePoseEstimate_sub=nh.subscribe("current_state_est", 1, &Navigation::dronePoseEstimateCallback, this);
    // Set up a publisher to send on "explored" topic with boolen message type
    explore_pub = nh.advertise<std_msgs::Bool>("explored", 1);

    if (!nh.getParam(ros::this_node::getName() + "/one_marker_strategy/setpointReachedThreshold", setpointReachedThreshold))
    {
        ROS_ERROR("Could not read setpointReachedThreshold from parameter server");
    }
}

int Navigation::getNextGateId()
{
    return gates.size();
}

int Navigation::getLeftArucoId(int gateId)
{
    return gateId * 4;
}

int Navigation::getRightArucoId(int gateId)
{
    return gateId * 4 + 2;
}

void Navigation::addGate(Gate gate)
{
    gates.push_back(gate);
}

bool Navigation::nextGateIsVisible(int leftId, int rightId, const aruco_msgs::MarkerArray::ConstPtr& msg)
{
    bool foundRightId = false;
    bool foundLeftId = false;
    for (int i = 0; i < msg->markers.size(); i++)
    {
        if (msg->markers[i].id == rightId)
        {
            foundRightId = true;
        }
        if (msg->markers[i].id == leftId)
        {
            foundLeftId = true;
        }
    }
    return foundLeftId && foundRightId;
}
bool Navigation::nextLeftArucoIsVisible(int leftId, const aruco_msgs::MarkerArray::ConstPtr& msg)
{
    bool foundLeftId = false;
    for (int i = 0; i < msg->markers.size(); i++)
    {
        if (msg->markers[i].id == leftId)
        {
            foundLeftId = true;
        }
    }
    return foundLeftId;
}
bool Navigation::nextRightArucoIsVisible(int rightId, const aruco_msgs::MarkerArray::ConstPtr& msg)
{
    bool foundRightId = false;
    for (int i = 0; i < msg->markers.size(); i++)
    {
        if (msg->markers[i].id == rightId)
        {
            foundRightId = true;
        }
    }
    return foundRightId;
}
void Navigation::arucoCallback(const aruco_msgs::MarkerArray::ConstPtr& msg)
{   
    if (finished)
    {   
        return;
    }
    if (timeout == 0)
    {
            
        // define some variables
        int gateId;
        int leftArucoId;
        int rightArucoId;
        bool gateIsVisible;
        bool leftArucoIsVisible;
        bool rightArucoIsVisible;
        tf::Vector3 leftPylon;
        tf::Vector3 rightPylon;
        // What is my next gate?
        gateId = getNextGateId();
        leftArucoId = getLeftArucoId(gateId);
        rightArucoId = getRightArucoId(gateId);
        if (gateId==10)
        {
            ROS_WARN("All gates are detected");
            sendToController(tf::Vector3(0,0,5), tf::Quaternion(0,0,0,1));
            sendToController(tf::Vector3(0,0,1), tf::Quaternion(0,0,0,1));
            finished = true;
            std_msgs::Bool message;
            message.data = true;
            explore_pub.publish(message);
            // export all gates to a file
            //open file for writing
            std::string filename = "/workspaces/autsys-projects-student-airrace/catkin_ws/src/gates.csv";
            std::ofstream myfile(filename, std::ofstream::out);
            //check if file was successfully opened for writing
            if (myfile.is_open())
            {
                for (int i = 0; i < gates.size(); i++)
                {
                    myfile << gates[i].getLeftPylon().getX() << ";" << gates[i].getLeftPylon().getY() << ";" << gates[i].getLeftPylon().getZ() << ";" << gates[i].getRightPylon().getX() << ";" << gates[i].getRightPylon().getY() << ";" << gates[i].getRightPylon().getZ() << ";" << gates[i].getCenter().getX() << ";" << gates[i].getCenter().getY() << ";" << gates[i].getCenter().getZ() << ";" << gates[i].getOrientation().getX() << ";" << gates[i].getOrientation().getY() << ";" << gates[i].getOrientation().getZ() << ";" << gates[i].getOrientation().getW() << ";" << gates[i].getAngle() << "\n";
            
                }
                myfile.close();
            }
            else
            {
                ROS_WARN("Unable to open file");
            }
          
            
        
        }
        // Check if the next gate is visible
        gateIsVisible = nextGateIsVisible(leftArucoId, rightArucoId, msg);
        leftArucoIsVisible = nextLeftArucoIsVisible(leftArucoId, msg);
        rightArucoIsVisible = nextRightArucoIsVisible(rightArucoId, msg);
        if (leftArucoIsVisible && rightArucoIsVisible)
        {
            // Create a gate
            for (int i = 0; i < msg->markers.size(); i++)
            {

                if(msg->markers[i].id == leftArucoId)
                {
                    leftPylon = tf::Vector3(msg->markers[i].pose.pose.position.x, msg->markers[i].pose.pose.position.y, msg->markers[i].pose.pose.position.z);
                }
                else if(msg->markers[i].id == rightArucoId)
                {
                    rightPylon = tf::Vector3(msg->markers[i].pose.pose.position.x, msg->markers[i].pose.pose.position.y, msg->markers[i].pose.pose.position.z);
                }
                
            }
            addGate(Gate(leftPylon, rightPylon));
            sendToController(gates[gateId].getCenter(), gates[gateId].getOrientation());
            // 40 -> 1s
            timeout = 80;
            // 400 -> 10s
            //notDetectedTimeout = 400;
            ROS_INFO("Gate %d is detected. Orientation: %f", gateId, gates[gateId].getAngle());
            searchStrategyOneMarkerActive = false;
            counterSearchSteps = 0;

        }
        else if((leftArucoIsVisible == 1 && rightArucoIsVisible == 0) || (leftArucoIsVisible == 0 && rightArucoIsVisible == 1))
        {
            if(!searchStrategyOneMarkerActive)
            {

                for (int i = 0; i < msg->markers.size(); i++)
                {

                    if(msg->markers[i].id == leftArucoId)
                    {
                        // print ou the distance to the marker
                        double distance = (currentDronePositionWorld.distance(tf::Vector3(msg->markers[i].pose.pose.position.x, msg->markers[i].pose.pose.position.y, msg->markers[i].pose.pose.position.z)));
                        //ROS_INFO("Distance to left marker: %f", distance);
                        if (distance <7.0)
                        {    
                            ROS_INFO("Search strategy one marker active");
                            stepAround(msg,i);
                            searchStrategyOneMarkerActive = true;
                            timeout = 80;                     
                        }

                    }
                    else if(msg->markers[i].id == rightArucoId)
                    {
                        ROS_INFO("Search strategy one marker active");
                        stepAround(msg,i);    
                        searchStrategyOneMarkerActive = true;
                        timeout = 80;              
                    }
                    
                }
            }
        }
    }
    else
    {   // count down the timeout
        timeout--;
    }




    /*
    **************************************************************
    // time-based search strategy - is an alternative to the distance-based search strategy

    if (notDetectedTimeout == 0)
    {
        // since 10 seconds no gate was detected, we fly 10 meters forward
        stepForward();
        
        sendToController(targetPositionWorld, targetOrientationWorld);
        notDetectedTimeout = 400;
    }
    else
    {
        notDetectedTimeout--;
        ROS_INFO("Not detected timeout: %d", notDetectedTimeout);
    }
    ***************************************************************
    */

    //ROS_INFO("Controller error in meters: %f", currentDronePositionWorld.distance(lastPositionSetpointWorld));   

    if(currentDronePositionWorld.distance(lastPositionSetpointWorld) < setpointReachedThreshold)
    {
        reachedTimeout --;
        if (reachedTimeout == 0)
        {
            stepForward();         
            counterSearchSteps++;       
        }

    }
    // outside the range around the setpoint
    else
    {
        // reset timeout to 1s
        reachedTimeout = 40;
    }

    

}

// set target and orientation of the drone to the controller
void Navigation::sendToController(tf::Vector3 target, tf::Quaternion orientation)
{
    // save last setpoint : orientation and position
    lastPositionSetpointWorld = target;    
    lastOrientationSetpointWorld = orientation;
    tf::Transform desired_pose(tf::Transform::getIdentity());
    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;
    velocity.linear.x = velocity.linear.y = velocity.linear.z = 0.5;
    velocity.angular.x = velocity.angular.y = velocity.angular.z = 0.5;
    acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0.2;
    acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0.2;
    desired_pose.setOrigin(target);
    desired_pose.setRotation(orientation);
    trajectory_msgs::MultiDOFJointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "true_body" ignored by controller
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
    msg.points[0].accelerations.resize(1);
    msg.points[0].velocities[0] = velocity;
    msg.points[0].accelerations[0] = acceleration;
    waypoint_pub.publish(msg);
    //ROS_INFO("Send to controller: x: %f, y: %f, z: %f", target.x(), target.y(), target.z());
}

void Navigation::convertToWorld(tf::Vector3 &worldPosition, tf::Quaternion &worldOrientation,
                                 tf::Vector3 &position, tf::Quaternion &orientation,
                                 tf::Vector3 &newWorldPosition, tf::Quaternion &newWorldOrientation)
{
    // calculate the position of the drone in the world frame, when moved by position and orientation in the body frame
    /*
    forward kinematics
    */

    // transformation from true_body to new setpoint for controller is 4x4 matrix (3x3 rotation matrix + 3x1 translation vector)
    tf::Transform transform;
    transform.setOrigin(position);
    transform.setRotation(orientation);
    // transformation from world to true_body is 4x4 matrix (3x3 rotation matrix + 3x1 translation vector)
    tf::Transform worldTransform;
    worldTransform.setOrigin(worldPosition);
    worldTransform.setRotation(worldOrientation);
    // transformation from world to new setpoint for controller is 4x4 matrix (3x3 rotation matrix + 3x1 translation vector)
    tf::Transform newWorldTransform = worldTransform * transform;
    newWorldPosition = newWorldTransform.getOrigin();
    newWorldOrientation = newWorldTransform.getRotation();
}

void Navigation::dronePoseEstimateCallback(const nav_msgs::Odometry &msg)
{
    currentDronePositionWorld = tf::Vector3(msg.pose.pose.position.x, 
                                                msg.pose.pose.position.y, 
                                                msg.pose.pose.position.z);
}
void Navigation::stepForward()
{
    tf::Vector3 targetPositionLocal = tf::Vector3(5, 0, 0);  //setpoint position in local frame
    tf::Quaternion targetOrientationLocal = tf::Quaternion(0, 0, 0, 1); // setpoint orientation in local frame
    tf::Vector3 targetPositionWorld= tf::Vector3(0, 0, 0);  //setpoint position in world frame
    tf::Quaternion targetOrientationWorld = tf::Quaternion(0, 0, 0, 1); // setpoint orientation in world frame
    convertToWorld(lastPositionSetpointWorld, lastOrientationSetpointWorld,
                        targetPositionLocal, targetOrientationLocal,
                        targetPositionWorld, targetOrientationWorld);

    
    sendToController(targetPositionWorld, targetOrientationWorld);
}
void Navigation::stepAround(const aruco_msgs::MarkerArray::ConstPtr& msg,int index)
{
    // define an eigen vector 3x1 in aruco frame

    Eigen::Vector3d targetPositionLocal;
    double x_stepAround = 0.0;
    double y_stepAround = 0.0;
    targetPositionLocal << 5.0, 0.0 , 20.0;
    // // Define a 4x4 transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // // Define a rotation matrix    // transoform quaternion to rotation matrix

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond q_world_to_aruco;
    q_world_to_aruco.x() = msg->markers[index].pose.pose.orientation.x;
    q_world_to_aruco.y() = msg->markers[index].pose.pose.orientation.y;
    q_world_to_aruco.z() = msg->markers[index].pose.pose.orientation.z;
    q_world_to_aruco.w() = msg->markers[index].pose.pose.orientation.w;

    // define a new quaternion
    Eigen::Quaterniond q_aruco_to_target;
    q_aruco_to_target.x() = -0.5;
    q_aruco_to_target.y() = 0.5;
    q_aruco_to_target.z() = 0.5;
    q_aruco_to_target.w() = 0.5;
    R = q_world_to_aruco.toRotationMatrix();

    // compose two quaternions
    Eigen::Quaterniond q_world_to_target = q_world_to_aruco * q_aruco_to_target;

    // // Define a translation vector
    Eigen::Vector3d t;
    t << msg->markers[index].pose.pose.position.x, msg->markers[index].pose.pose.position.y, msg->markers[index].pose.pose.position.z;

    //print out the result
    // // Define a 4x4 transformation matrix
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    
    // change x_stepAround and y_stepAroudn according to the marker yaw angle with 20 meters distance using trigonometry
    x_stepAround = 20 * cos(q_world_to_aruco.toRotationMatrix().eulerAngles(0, 1, 2)(2));
    y_stepAround = 20 * sin(q_world_to_aruco.toRotationMatrix().eulerAngles(0, 1, 2)(2));

    // // Define a 4x1 vector
    Eigen::Vector4d p;
    p << targetPositionLocal(0), targetPositionLocal(1), targetPositionLocal(2), 1;
    // // Perform the transformation
    Eigen::Vector4d new_p = T * p;

    // print out yaw angle from quaternion
    // convert eigen vector to tf vector
    tf::Vector3 targetPositionWorld = tf::Vector3(new_p(0), new_p(1), new_p(2));
    tf:: Vector3 targetPostionWorld2 = tf::Vector3(0, 0, 0);
    tf::Quaternion targetOrientationWorld2= tf::Quaternion(0, 0, 0, 1);
    
    tf::Quaternion targetOrientationWorld = tf::Quaternion(q_world_to_target.x(), q_world_to_target.y(), q_world_to_target.z(), q_world_to_target.w());
    sendToController(targetPositionWorld, targetOrientationWorld);

    tf::Vector3 worldPosition = tf::Vector3(t(0), t(1), t(2));
    tf::Quaternion worldOrientation = tf::Quaternion(q_world_to_aruco.x(), q_world_to_aruco.y(), q_world_to_aruco.z(), q_world_to_aruco.w());
    tf::Vector3 targetPositionLocal2 = tf::Vector3(0, 0, 10);
    tf::Quaternion targetOrientationLocal2 = tf::Quaternion(0, 0, 0, 1);

    // we can use this function to convert the target position and orientation from local frame to world frame
    // convertToWorld(worldPosition, worldOrientation,
    //                     targetPositionLocal2, targetOrientationLocal2,
    //                     targetPostionWorld2, targetOrientationWorld2);
    
    // print out the result to check if the conversion is correct

    // ROS_INFO("targetPositionWorld: %f, %f, %f", targetPositionWorld.x(), targetPositionWorld.y(), targetPositionWorld.z());
    // ROS_INFO("yaw angle: %f", tf::getYaw(targetOrientationWorld)*180/M_PI);
    // ROS_INFO("aruco position: %f, %f, %f", t(0), t(1), t(2));
    // ROS_INFO("aruco orientation: %f, %f, %f, %f", q_world_to_aruco.x(), q_world_to_aruco.y(), q_world_to_aruco.z(), q_world_to_aruco.w());
    // ROS_INFO("rotation matrix: %f, %f, %f", R(0,0), R(0,1), R(0,2));
    // ROS_INFO("rotation matrix: %f, %f, %f", R(1,0), R(1,1), R(1,2));
    // ROS_INFO("rotation matrix: %f, %f, %f", R(2,0), R(2,1), R(2,2));
    // ROS_INFO("targetPositionWorld2: %f, %f, %f", targetPostionWorld2.x(), targetPostionWorld2.y(), targetPostionWorld2.z());
    // ROS_INFO("targetOrientationWorld2: %f, %f, %f, %f", targetOrientationWorld2.x(), targetOrientationWorld2.y(), targetOrientationWorld2.z(), targetOrientationWorld2.w());
    

}