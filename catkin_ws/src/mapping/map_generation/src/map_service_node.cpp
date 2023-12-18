#include <ros/ros.h>
#include <map_generation/CreateMap.h>
#include <geometry_msgs/Vector3.h>
#include <algorithm>
#include <fstream>
#include <numeric>

#define GATES 10

class Map_service
{
private:
  // Define the dimension of the array of vectors hosting the coordinates detected through time
  std::vector<float> gatex[GATES * 4];
  std::vector<float> gatey[GATES * 4];
  std::vector<float> gatez[GATES * 4];

  std::vector<float> filt_datax;
  std::vector<float> filt_datay;
  std::vector<float> filt_dataz;

  float wrong_id = 0;
  float detections = 0;
  float error_rate = 0.0f;
  int detection_id[GATES * 4]{0};
  int outliers[GATES * 4]{0};
  // Threshold for outliers
  float zscore_threshold = 3.0f;
  bool outlier_detection;

  geometry_msgs::Vector3 mean[GATES * 4];
  geometry_msgs::Vector3 variance[GATES * 4];
  geometry_msgs::Vector3 sum;
  geometry_msgs::Vector3 sumSquaredDiff;
  geometry_msgs::Vector3 stdev;

  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("map", &Map_service::gateout, this);

public:
  bool gateout(map_generation::CreateMap::Request &req, map_generation::CreateMap::Response &res)
  { 

    if (!nh.getParam(ros::this_node::getName() + "/outlier_analysis", outlier_detection))
    {
        ROS_WARN("Could not read outlier_analysis flag from map_generation launch file");
    }

    if (req.gateid >= 0 && req.gateid <= (GATES*4-1))
    {
      // check if req is valid
      if (req.gatein.x == 0.0f && req.gatein.y == 0.0f && req.gatein.z == 0.0f)
      {
        ROS_INFO("Invalid request");
      }
      else{

        detections += 1;
        detection_id[req.gateid] += 1;

        gatex[req.gateid].push_back(req.gatein.x);
        gatey[req.gateid].push_back(req.gatein.y);
        gatez[req.gateid].push_back(req.gatein.z);
      }
      meanAndVariance(gatex[req.gateid], gatey[req.gateid], gatez[req.gateid], mean[req.gateid], variance[req.gateid]);

      if(outlier_detection == true && std::min({gatex[req.gateid].size(), gatey[req.gateid].size(), gatez[req.gateid].size()}) > 1){

        outliers[req.gateid] = 0;
        removeOutliers(gatex[req.gateid], gatey[req.gateid], gatez[req.gateid], mean[req.gateid], variance[req.gateid], filt_datax, filt_datay, filt_dataz, req.gateid);
        meanAndVariance(filt_datax, filt_datay, filt_dataz, mean[req.gateid], variance[req.gateid]);
      }
        
      for (int i = 0; i <= (GATES*4-1); i++)
      {
      res.out_est[i] = mean[i];
      }

      exportmap(mean, variance, (GATES*4-1));
    }
    else
    {
      ROS_INFO("Wrong detected marker id: %i", req.gateid);
      wrong_id += 1;
    };
    return true;
  }

  // Export the map to a file
  void exportmap(geometry_msgs::Vector3 mean[], geometry_msgs::Vector3 var[], int n)
  {

    // create an output stream object and open the file
    std::ofstream outputFile("/workspaces/autsys-projects-student-airrace/catkin_ws/src/mapping/map_generation/track");

    // check if the file was opened successfully
    if (!outputFile.is_open())
    {
      ROS_WARN("Failed to open track file");
      return;
    }

    outputFile << "Marker | N° of detections |                       Estimation " << std::endl;

    for (int i = 0; i <= n; i++)
    {
      // write the data to the file
      outputFile << "id: " << i << "           " << detection_id[i] << "            x: " << mean[i].x << " (" << variance[i].x << ") "
                 << " y: " << mean[i].y << " (" << variance[i].y << ") "
                 << " z: " << mean[i].z << " (" << variance[i].z << ") " << "   n°outliers:" << outliers[i] << std::endl;
    }

    // Print the overall number of detections
    outputFile << "Overall number of marker detections until now: " << detections << std::endl;
    // Print the number of wrong detected markers
    outputFile << "Number of detected markers out of range until now: " << wrong_id << std::endl;
    // Calculate the percentage of wrong detected markers
    error_rate = wrong_id / detections * 100;
    outputFile << "Percentage of wrong detected markers: " << error_rate << " %" << std::endl;

    // close the file
    outputFile.close();
  }

  // Calculate the mean and variance of input vector
  void meanAndVariance(std::vector<float> datax, std::vector<float> datay, std::vector<float> dataz, geometry_msgs::Vector3 &mean, geometry_msgs::Vector3 &variance)
  {

    // Compute the mean and variance over the detected coordinates
    if (std::min({datax.size(), datay.size(), dataz.size()}) > 0)
    {

      sum.x = 0.0f;
      sum.y = 0.0f;
      sum.z = 0.0f;

      // Calculate the variance
      sumSquaredDiff.x = 0.0f;
      sumSquaredDiff.y = 0.0f;
      sumSquaredDiff.z = 0.0f;

      for (int j = 0; j < std::min({datax.size(), datay.size(), dataz.size()}); j++)
      {

        sum.x += datax[j];
        sum.y += datay[j];
        sum.z += dataz[j];
      }

      mean.x = sum.x / std::min({datax.size(), datay.size(), dataz.size()});
      mean.y = sum.y / std::min({datax.size(), datay.size(), dataz.size()});
      mean.z = sum.z / std::min({datax.size(), datay.size(), dataz.size()});

      for (int k = 0; k < std::min({datax.size(), datay.size(), dataz.size()}); k++)
      {
        float diffx = datax[k] - mean.x;
        sumSquaredDiff.x += diffx * diffx;

        float diffy = datay[k] - mean.y;
        sumSquaredDiff.y += diffy * diffy;

        float diffz = dataz[k] - mean.z;
        sumSquaredDiff.z += diffz * diffz;
      }
      variance.x = sumSquaredDiff.x / std::min({datax.size(), datay.size(), dataz.size()});
      variance.y = sumSquaredDiff.y / std::min({datax.size(), datay.size(), dataz.size()});
      variance.z = sumSquaredDiff.z / std::min({datax.size(), datay.size(), dataz.size()});
    }
    else
    {
      mean.x = 0.0f;
      mean.y = 0.0f;
      mean.z = 0.0f;

      variance.x = 0.0f;
      variance.y = 0.0f;
      variance.z = 0.0f;
    }
  }

  void removeOutliers(std::vector<float> datax, std::vector<float> datay, std::vector<float> dataz, geometry_msgs::Vector3 mean, geometry_msgs::Vector3 variance, std::vector<float> &filt_datax, std::vector<float> &filt_datay, std::vector<float> &filt_dataz, int gateid) {

    stdev.x = std::sqrt(variance.x);
    stdev.y = std::sqrt(variance.y);
    stdev.z = std::sqrt(variance.z);

    filt_datax.clear();
    filt_datay.clear();
    filt_dataz.clear();


    // Remove data points that are more than 3 standard deviations from the mean
    for (int i = 0; i < datax.size(); i++) {
        if (std::abs(datax[i] - mean.x) < (zscore_threshold * stdev.x) && std::abs(datay[i] - mean.y) < (zscore_threshold * stdev.y) && std::abs(dataz[i] - mean.z) < (zscore_threshold * stdev.z)) {
            filt_datax.push_back(datax[i]);
            filt_datay.push_back(datay[i]);
            filt_dataz.push_back(dataz[i]);
        }else{
          outliers[gateid] += 1;
        }
    }
  }
};

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "map_service");

  Map_service map_service;

  // Spin forever
  ros::spin();

  return 0;
}