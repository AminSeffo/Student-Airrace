//Autor: Amin Seffo
//Date: 04.03.2023

#include "navigation.h"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "navigation");

    for (int i = 0; i < 50; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }
    // Create an instance of the navigation class
    Navigation navigation;

    // Spin forever
    ros::spin();


    return 0;
}
