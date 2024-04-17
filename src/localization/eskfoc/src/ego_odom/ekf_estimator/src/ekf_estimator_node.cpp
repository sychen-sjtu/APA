//
// Created by ubuntu on 24-2-26.
//
#include "ros/ros.h"
#include "ekf_estimator.h"

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_estimator");
    ros::NodeHandle n;
    EkfEstimator estimator(n);
    estimator.RosNodeRegistration(n);

    ros::spin();
}
