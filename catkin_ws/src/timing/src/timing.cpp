#include <timing.h>
#include <fstream>
#include <math.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


#define MAX_FLIGHT_TIME 250.0
Timing::Timing(ros::NodeHandle nh_) : nh(nh_), numberCalls(0), minWrench(10000.0, 10.0, 10.0, 10.0), maxWrench(-10000, -10.0, -10.0, -10.0),
            numberCallsPropSpeeds(0), flightDuration(MAX_FLIGHT_TIME), referencePositions(), reachedPositionNTimes(), positionSequence(),
            droneAtGoalPosition(), stopPositions(), receivedTrajectory(0) {

    sub_dronePosition = nh.subscribe("current_state_est", 5, &Timing::checkDronePosition, this);
    LapStart_time = ros::Time::now();
    LastGateTime = ros::Time::now();
    lastGatePassed = -1;
    readReferencePositions();    
}

void Timing::readReferencePositions() {
    ROS_INFO("Reading reference positions...");
    referencePositions.resize(10);
    reachedPositionNTimes.resize(referencePositions.size());
    droneAtGoalPosition.resize(referencePositions.size());
    for(size_t i = 1; i <= referencePositions.size(); i++) {
        std::string gateName = "Gate_" + std::to_string(i);
        std::vector<double> pos;
        std::vector<double> posRotated{0,0,0};
        double orientation=0;

        //declate int variable orientation
        nh.getParam("/timing_node/Gates/"+gateName+"/pos", pos);
        nh.getParam("/timing_node/Gates/"+gateName+"/orientation", orientation);
        //caclulate position from unity frame to ros and rotate about oriantation around z-axis
        posRotated[0] = pos[0] - 5.0 * std::sin((orientation / 180.0) * ((double) M_PI));
        posRotated[1] = pos[2] - 5.0 * std::cos((orientation / 180.0) * ((double) M_PI));
        posRotated[2] = pos[1];    
        referencePositions[i-1] << posRotated[0], posRotated[1], posRotated[2];
        ROS_INFO("Position: %f, %f, %f", referencePositions[i-1][0], referencePositions[i-1][1], referencePositions[i-1][2]);
    }
    ROS_INFO("Reference positions read!");
}

void Timing::checkRunTime(const ros::TimerEvent& t) {
    ros::Time actual_time = ros::Time::now();
    double time = (actual_time - startup_time).toSec();
    if(time > flightDuration) {
        writeTestResult();
        ros::shutdown();
    }
}

static double signed_pow2(double val) {
    return val>0?pow(val,2):-pow(val,2);
}

void Timing::checkDronePosition(const nav_msgs::Odometry& cur_state) {
    Eigen::Vector3d dronePosition;
    dronePosition << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
    checkGoalPositions(dronePosition);
}

void Timing::checkGoalPositions(Eigen::Vector3d dronePosition) {
    for(size_t i = 0; i < referencePositions.size(); i++) {
        if(abs((dronePosition-referencePositions[i]).norm()) < 5) {
            if(lastGatePassed == i-1) {
                ROS_INFO("Drone reached Gate %ld", i+1);
                ROS_INFO("Time since last Gate: %f", (ros::Time::now() - LastGateTime).toSec());
                LastGateTime = ros::Time::now();
            }
            if(i==0 && lastGatePassed == 9){
                ROS_INFO("LapComplete");
                ROS_INFO("Time since last Lap: %f", (ros::Time::now() - LapStart_time).toSec());
                LapStart_time = ros::Time::now();
            }
            lastGatePassed = i;
        }
    }
}

void Timing::writeTestResult() {
    std::ofstream results("../../../results.txt");
    if(!results.is_open()) {
        ROS_ERROR_STREAM("No Results available!");
        return;
    }
    if(numberCallsPropSpeeds == 0) {
        maxWrench = {-std::nan(""),-std::nan(""),-std::nan(""),-std::nan("")};
        minWrench = {std::nan(""),std::nan(""),std::nan(""),std::nan("")};
    }
    results << "##########################\nResult Report:\n"
        << "Recorded flight time: " << flightDuration << std::endl
        << "Tested drone positions: " << numberCalls << std::endl
        << "Tested rotor speed commands: " << numberCallsPropSpeeds << std::endl
        << "Received (elementwise) maximum wrench: " << maxWrench[0] << ", " << maxWrench[1] << ", " << maxWrench[2] << ", " << maxWrench[3] << "\n"
        << "Received (elementwise) minimum wrench: " << minWrench[0] << ", " << minWrench[1] << ", " << minWrench[2] << ", " << minWrench[3] << "\n"
        << "Goal sequence: ";
    //print sequence
    for(int i = 0; i < positionSequence.size(); i++) {
        int pos = positionSequence[i]+1;
        results << pos;
        if(i != positionSequence.size()-1) {
            results << ", ";
        }
    }
    //print reached positions
    results << "\nCounted drone transitions for all " << referencePositions.size() << " goals:\n";
    for(int i = 0; i < referencePositions.size(); i++) {
        results << "    Counted transitions for [" << referencePositions[i][0] << ", " << referencePositions[i][1] << ", " << referencePositions[i][2]
            << "]: " << reachedPositionNTimes[i] << std::endl;
    }
    results << "Reported number of drone positions with 0 velocity: " << stopPositions.size() << std::endl;
    for(auto iPos : stopPositions) {
        results << "    [" << iPos[1] << ", " << iPos[2] << ", " << iPos[3] << "] at time: " << iPos[0] << std::endl;
    }
    results << "##########################\n\n";
    results.close();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "timing_node");
    ros::NodeHandle nh;
    Timing test(nh);
    ros::spin();
}