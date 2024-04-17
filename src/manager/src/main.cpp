#include <ros/ros.h>
#include "manager/manger.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;

    APA::Manager manager(nh);
    ros::spin();
    return 0;
}