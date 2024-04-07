#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <parking_slot_detection/gcn_parking.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;

    std::string image_path;
    if(!nh.getParam("client_node/image_path", image_path)){
        std::cout << "error!!! can not get image_path: " << image_path << std::endl;
    }
    
    // 读取图像
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::resize(image, image, cv::Size(512, 512));

    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");

    // 创建请求和响应消息
    parking_slot_detection::gcn_parking srv;
    srv.request.image_data = *image_msg;
    ros::Rate rate(1);

    cv::namedWindow("detected_results");

    while(ros::ok()){
        // 发送请求
        if (client.call(srv)) {
            cv::Mat image_vis = image.clone();
            // 成功接收响应
            cv::Scalar color(255, 0, 0);
            for (size_t i = 0; i < srv.response.point0_x.size(); ++i) {
                int point0_x = srv.response.point0_x[i];
                int point0_y = srv.response.point0_y[i];
                int point1_x = srv.response.point1_x[i];
                int point1_y = srv.response.point1_y[i];
                int type = srv.response.types[i];
                cv::circle(image_vis, cv::Point(point0_x, point0_y), 3, color, 2);
                cv::circle(image_vis, cv::Point(point1_x, point1_y), 3, color, 2);
            }
            cv::imshow("detected_results", image_vis);
            cv::waitKey(5);
        } else {
            ROS_ERROR("Failed to call service");
        }
        rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}