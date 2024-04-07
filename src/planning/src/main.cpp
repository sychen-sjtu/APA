#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "planning/planner.h"
#include <parking_slot_detection/gcn_parking.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;

    Planner planner(nh);
    ros::spin();
    return 0;
}