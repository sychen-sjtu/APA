#pragma once
#ifndef PLANNER_H

#include <ros/ros.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "parking_slot_detection/gcn_parking.h"
#include "common/common_struct.hpp"


class Planner {
    public:
        using boost_point = boost::geometry::model::d2::point_xy<double>;
        using boost_polygon = boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>;

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
        std::vector<boost_polygon> get_obstacles_from_parking_slot(ParkingSlot &parking_slot, double road_width, double min_obstacle_length);
        bool polygon_validate_check(Eigen::Vector3d pose, std::vector<boost_polygon> & obstacle_list);
        
        void draw_rectangle(cv::Mat &image, double center_x, double center_y, double length, double width, double theta, cv::Scalar color);

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
        
        // opml的RS规划器
        ompl::base::StateSpacePtr shotptr;

        int frame_cnt = 0;
        std::string output_dir;
        bool if_save_results;
  
};

#endif /* PLANNER_H */
