#pragma once
#ifndef PLANNER_H

#include <ros/ros.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>

#include "parking_slot_detection/gcn_parking.h"


class Planner {
    public:
        struct VehicleParam{
            double K_f;
            double K_r;
            double L_f;
            double L_r;
            double L;
            double vehicle_length;
            double vehicle_width;
            double vehicle_r_min;
        };

        struct ParkingSlot{
            Eigen::Vector2d center;     // 车位中心坐标
            double theta;               // 车位朝向
            double length;              // 车位长度
            double width;               // 车位宽度
        };

        struct Trajectory{
            std::vector<Eigen::Vector3d> trajectory_points;
            double step_length;
        };

        Planner(ros::NodeHandle &nh);
        bool call_gcn_server(cv::Mat &avm_image, std::vector<std::pair<cv::Point, cv::Point>> &detected_point);
        ~Planner();
    private:
        void avm_image_call_back(const sensor_msgs::ImageConstPtr& msg);
        Eigen::Vector2d point_front_img_to_vehicle(cv::Point img_point);
        cv::Point point_front_vehicle_to_img(Eigen::Vector2d vehicle_point);
        ParkingSlot get_parking_slot(std::pair<cv::Point, cv::Point> parking_slot_points);
        
        // plan base on geometry
        double get_r_fc(double s, double b1, double b2, double b3);
        double get_r_sc(double s, double b1, double b2, double b3);
        Trajectory geometry_plan(Eigen::Vector3d start_pose, ParkingSlot parking_slot);
        
        double normalize_angle(double angle);
        void draw_rectangle(cv::Mat &image, double center_x, double center_y, double length, double width, double theta);

        ros::NodeHandle nh_;
        ros::ServiceClient client_;
        ros::Subscriber avm_image_sub_;

        std::string avm_image_topic_;
        std::string gcn_server_topic_;
        
        cv::Size target_input_size_;

        double per_pixel_length_;
        cv::Size avm_image_size_;
        double avm_image_width_, avm_image_height_;

        double parking_slot_length, parking_slot_width;

        VehicleParam vehicle_param;

        double trajectory_step_length;
  
};

#endif /* PLANNER_H */
