#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <map_generation/CreateMap.h>
#include <geometry_msgs/Vector3.h>

class Map_client
{
    private:

      ros::NodeHandle nh;
      // Set up a subscriber to receive transform messages from aruco_node
      ros::Subscriber aruco_marker = nh.subscribe("/aruco_marker_publisher/markers", 10, &Map_client::storingCallback,this);

    public:

      // Define the callback function for the subscriber
      void storingCallback(const aruco_msgs::MarkerArray::ConstPtr& msg){

      // Create a service client to call the service
      ros::ServiceClient client = nh.serviceClient<map_generation::CreateMap>("map");

      // Store the received message in variable current_marker and send it through the service together with gate's id
      for (int i = 0; i < msg->markers.size(); i++){

        // Create a service request message
        map_generation::CreateMap srv;
        geometry_msgs::Vector3 current_marker;

        current_marker.x = msg->markers[i].pose.pose.position.x;
        current_marker.y = msg->markers[i].pose.pose.position.y;
        current_marker.z = msg->markers[i].pose.pose.position.z;

        //Send all the coordinates through the service
        srv.request.gatein = current_marker;
        srv.request.gateid = msg->markers[i].id;

        // Call the service and check if it was successful
        if (client.call(srv)){
          ROS_INFO("Service call successful!");
          continue;
        }else{
          ROS_WARN("Failed to call service!");
          continue;
        }
      }
      }
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "map_client");

    Map_client map_client;

    // Spin forever
    ros::spin();

    return 0;
}

