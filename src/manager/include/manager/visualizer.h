#ifndef _VISUALIZER_HEADER
#define _VISUALIZER_HEADER

#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "common/common_struct.h"

#include <Eigen/Core>

namespace APA{
    class Visualizer
    {
        public:
            Visualizer();
            Visualizer(ros::NodeHandle &nh);

            void publish_parking_slots(std::vector<ParkingSlot> & parking_slots);
        
        private:
            ros::NodeHandle nh_;
            ros::Publisher parking_slots_pub_;
    };
    
}

#endif