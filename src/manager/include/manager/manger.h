#ifndef MANGER
#define MANGER

#include "ros/ros.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include "common/common_struct.h"
#include "common/math_utils.h"
#include "manager/parking_slot_manager.h"
#include "manager/visualizer.h"
// #include "planning/planner.h"

#include "parking_slot_detection/gcn_parking.h"

namespace APA{

    class Manager{
        public:
            Manager(ros::NodeHandle &nh);

            ros::NodeHandle nh_;
            ros::Subscriber odom_sub_;
            ros::Subscriber avm_image_sub_;

            ros::ServiceClient parking_slot_detection_client_;
            
            ParkingSlotManager parking_slot_manager;
            Visualizer visualizer;
            
            void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void avmImageCallback(const sensor_msgs::Image::ConstPtr& msg);

            bool call_gcn_server(cv::Mat &avm_image, std::vector<std::pair<cv::Point, cv::Point>> &detected_point);

            Eigen::Vector2d point_front_img_to_vehicle(cv::Point img_point);
            cv::Point point_front_vehicle_to_img(Eigen::Vector2d vehicle_point);
            bool point_front_vehicle_to_base(Eigen::Vector2d &vehicle_point, Eigen::Vector2d &base_point);
            bool point_front_img_to_base(cv::Point img_point, Eigen::Vector2d &base_point);
            bool point_front_img_to_world(cv::Point img_point, Eigen::Vector2d &world_point);

        private:
            int avm_image_frame_cnt = 0;

            cv::Size nn_target_input_size_;
            double avm_per_pixel_length_;
            cv::Size avm_image_size_;
            double avm_image_width_, avm_image_height_;

            double parking_slot_length, parking_slot_width;

            VehicleParam vehicle_param;

            // localization
            bool localization_init_;
            Eigen::Vector2d self_position_; // x, y in world
            double self_yaw_;
            Eigen::Matrix2d ratation_from_base_to_world_;



        
    };

}

#endif