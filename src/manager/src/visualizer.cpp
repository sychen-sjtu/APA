#include "manager/visualizer.h"


namespace APA{
    Visualizer::Visualizer(){}

    Visualizer::Visualizer(ros::NodeHandle &nh){
        nh_ = nh;
        parking_slots_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("parking_slots", 10);
    }

    void Visualizer::publish_parking_slots(std::vector<ParkingSlot> & parking_slots){
        visualization_msgs::MarkerArray parking_slot_markers;
        for(auto parking_slot : parking_slots){
            // 计算车位的角点
            double parking_theta = parking_slot.theta;
            double half_length = parking_slot.length / 2.0;
            double half_width = parking_slot.width / 2.0;
            Eigen::Vector2d forward_vector = half_length * Eigen::Vector2d(std::cos(parking_theta), std::sin(parking_theta));
            Eigen::Vector2d left_vector = half_width * Eigen::Vector2d(std::cos(parking_theta + M_PI/2.0), std::sin(parking_theta + M_PI/2.0));
            
            Eigen::Vector2d left_forward_point = parking_slot.center + forward_vector + left_vector;
            Eigen::Vector2d right_forward_point = parking_slot.center + forward_vector - left_vector;
            Eigen::Vector2d left_back_point = parking_slot.center - forward_vector + left_vector;
            Eigen::Vector2d right_back_point = parking_slot.center - forward_vector - left_vector;

            Eigen::Vector2d up_left_point;
            visualization_msgs::Marker point1, point2;
            // 设置第一个点
            point1.header.frame_id = "world";
            point1.type = visualization_msgs::Marker::SPHERE;
            point1.pose.position.x = left_forward_point[0];
            point1.pose.position.y = left_forward_point[1];
            point1.pose.position.z = 0.0;
            point1.scale.x = 0.1;
            point1.scale.y = 0.1;
            point1.scale.z = 0.1;
            point1.color.a = 1.0;
            point1.color.r = 0.0;
            point1.color.g = 1.0;
            point1.color.b = 0.0;
            parking_slot_markers.markers.push_back(point1);
            // 设置第二个点
            point2.header.frame_id = "world";
            point2.type = visualization_msgs::Marker::SPHERE;
            point2.pose.position.x = right_forward_point[0];
            point2.pose.position.y = right_forward_point[1];
            point2.pose.position.z = 0.0;
            point2.scale.x = 0.1;
            point2.scale.y = 0.1;
            point2.scale.z = 0.1;
            point2.color.a = 1.0;
            point2.color.r = 0.0;
            point2.color.g = 0.0;
            point2.color.b = 1.0;
            parking_slot_markers.markers.push_back(point2);
            // 设置线段
            geometry_msgs::Point vertex_point;
            vertex_point.z = 0.0;

            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "world";
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.scale.x = 0.1;  // 线段的宽度
            line_marker.color.a = 1.0;  // 完全不透明
            line_marker.color.r = 1.0;  // 红色
            vertex_point.x = left_back_point[0]; vertex_point.y = left_back_point[1];
            line_marker.points.push_back(vertex_point);
            vertex_point.x = left_forward_point[0]; vertex_point.y = left_forward_point[1];
            line_marker.points.push_back(vertex_point);
            vertex_point.x = right_forward_point[0]; vertex_point.y = right_forward_point[1];
            line_marker.points.push_back(vertex_point);
            vertex_point.x = right_back_point[0]; vertex_point.y = right_back_point[1];
            line_marker.points.push_back(vertex_point);

            parking_slot_markers.markers.push_back(line_marker);
        }
        parking_slots_pub_.publish(parking_slot_markers);
    }
}
