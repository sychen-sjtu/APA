#include "planning/planner.h"


Planner::Planner(ros::NodeHandle &nh){
    nh_ = nh;
    
    #define PARAM(name, var)                                    \
        do {                                                    \
            if (!nh_.getParam(name, var)) {                     \
                ROS_ERROR("missing parameter '" #name "'");     \
                return;                                         \
            }                                                   \
            else{                                               \
                std::cout << "get param " << name << " : " << var << std::endl;         \
            }                                                   \
        } while (0)

    std::string avm_image_config_name;
    // PARAM("avm_image_config_name", avm_image_config_name);
    PARAM("/planning_node/gcn_server_topic", gcn_server_topic_);
    PARAM("/planning_node/avm_image_topic", avm_image_topic_);
    std::cout << "avm_image_topic_: " << avm_image_topic_ << std::endl;

    client_ = nh_.serviceClient<parking_slot_detection::gcn_parking>(gcn_server_topic_);
    avm_image_sub_ = nh_.subscribe(avm_image_topic_, 1, &Planner::avm_image_call_back, this);

    target_input_size_ = cv::Size(512, 512);
    per_pixel_length_ = 0.01;
    avm_image_size_ = cv::Size(1024, 1024);
    avm_image_width_ = avm_image_size_.width * per_pixel_length_;
    avm_image_height_ = avm_image_size_.height * per_pixel_length_;

    parking_slot_length = 5.25;
    parking_slot_width = 2.25;

    vehicle_param.K_f = 0.74;
    vehicle_param.K_r = 0.96;
    vehicle_param.L_f = 1.56;
    vehicle_param.L_r = 1.34;
    vehicle_param.vehicle_length = vehicle_param.K_f + vehicle_param.K_r + vehicle_param.L_f + vehicle_param.L_r;
    vehicle_param.L = vehicle_param.L_f + vehicle_param.L_r;
    vehicle_param.vehicle_width = 2.0;
    vehicle_param.vehicle_r_min = 3.5;

    trajectory_step_length = 0.01;

    cv::namedWindow("detected_results");
}

Planner::~Planner(){
    cv::destroyAllWindows();
}

double Planner::normalize_angle(double angle){
    while(angle > M_PI){
        angle -= 2 * M_PI;
    }
    while(angle < -M_PI){
        angle += 2 * M_PI;
    }
    return angle;
}

void Planner::draw_rectangle(cv::Mat &image, double center_x, double center_y, double length, double width, double theta){
    // 计算矩形的旋转角度对应的正弦值和余弦值
    double sin_val = std::sin(theta);
    double cos_val = std::cos(theta);
    // 计算矩形的四个顶点坐标
    double x1 = center_x - length / 2 * cos_val - width / 2 * sin_val;
    double y1 = center_y - length / 2 * sin_val + width / 2 * cos_val;

    double x2 = center_x + length / 2 * cos_val - width / 2 * sin_val;
    double y2 = center_y + length / 2 * sin_val + width / 2 * cos_val;

    double x3 = center_x + length / 2 * cos_val + width / 2 * sin_val;
    double y3 = center_y + length / 2 * sin_val - width / 2 * cos_val;

    double x4 = center_x - length / 2 * cos_val + width / 2 * sin_val;
    double y4 = center_y - length / 2 * sin_val - width / 2 * cos_val;

    std::vector<cv::Point> rectangle_vertices_image;
    rectangle_vertices_image.emplace_back(point_front_vehicle_to_img(Eigen::Vector2d(x1, y1)));
    rectangle_vertices_image.emplace_back(point_front_vehicle_to_img(Eigen::Vector2d(x2, y2)));
    rectangle_vertices_image.emplace_back(point_front_vehicle_to_img(Eigen::Vector2d(x3, y3)));
    rectangle_vertices_image.emplace_back(point_front_vehicle_to_img(Eigen::Vector2d(x4, y4)));
    int point_num = rectangle_vertices_image.size();
    for(int i = 0; i < point_num; i++){
        cv::line(image, rectangle_vertices_image[i % point_num], rectangle_vertices_image[(i + 1) % point_num], cv::Scalar(0, 255, 0), 2);
    }
}

Eigen::Vector2d Planner::point_front_img_to_vehicle(cv::Point img_point){
    double vehicle_point_x = avm_image_height_ / 2.0 - img_point.y * per_pixel_length_;
    double vehicle_point_y = avm_image_width_ / 2.0 - img_point.x * per_pixel_length_;
    return Eigen::Vector2d(vehicle_point_x, vehicle_point_y);
}

cv::Point Planner::point_front_vehicle_to_img(Eigen::Vector2d vehicle_point){
    int img_x = static_cast<int>((avm_image_width_ / 2.0 - vehicle_point[1]) / per_pixel_length_);
    int img_y = static_cast<int>((avm_image_height_ / 2.0 - vehicle_point[0]) / per_pixel_length_);
    return cv::Point(img_x, img_y);
}

Planner::ParkingSlot Planner::get_parking_slot(std::pair<cv::Point, cv::Point> parking_slot_points){
    cv::Point point0 = parking_slot_points.first;
    cv::Point point1 = parking_slot_points.second;
    Eigen::Vector2d point0_in_vehicle = point_front_img_to_vehicle(point0);
    Eigen::Vector2d point1_in_vehicle = point_front_img_to_vehicle(point1);
    Eigen::Vector2d point0_point1_vec = point1_in_vehicle - point0_in_vehicle;
    Eigen::Vector2d point0_point1_vec_normalized = point0_point1_vec.normalized();
    Eigen::Vector2d in_line_center = (point0_in_vehicle + point1_in_vehicle) / 2.0;
    double parking_slot_theta = std::atan2(-point0_point1_vec_normalized[0], point0_point1_vec_normalized[1]); // 与二者连线垂直
    double parking_slot_center_x = in_line_center[0] - std::cos(parking_slot_theta) * parking_slot_length / 2.0;
    double parking_slot_center_y = in_line_center[1] - std::sin(parking_slot_theta) * parking_slot_length / 2.0;
    Eigen::Vector2d parking_slot_center(parking_slot_center_x, parking_slot_center_y);
    
    ParkingSlot parking_slot;
    parking_slot.center = parking_slot_center;
    parking_slot.theta  = parking_slot_theta;
    parking_slot.length = parking_slot_length;
    parking_slot.width = point0_point1_vec.norm();

    return parking_slot;
}

double Planner::get_r_fc(double s, double b1, double b2, double b3){
    double square_value = std::pow((std::pow(s - b2 + vehicle_param.K_r, 2) + std::pow(b1, 2) + b1 * vehicle_param.vehicle_width) / (2 * b1), 2) + std::pow(vehicle_param.L_r, 2);
    return std::sqrt(square_value);
}

double Planner::get_r_sc(double s, double b1, double b2, double b3){
    double square_value = std::pow(std::sqrt((s - b3) * (s - b3 - 2 * vehicle_param.K_f - 2 * vehicle_param.L)) - vehicle_param.vehicle_width / 2.0, 2) + std::pow(vehicle_param.L_r, 2);
    return std::sqrt(square_value);
}

Planner::Trajectory Planner::geometry_plan(Eigen::Vector3d start_pose, Planner::ParkingSlot parking_slot){
    // 规划分为两步
    Trajectory final_trajectory;
    final_trajectory.trajectory_points.clear();
    double b1 = (parking_slot.width - vehicle_param.vehicle_width)/2;
    double b2 = parking_slot.length;  // 旁边障碍物的长度，现在直接把旁边车位视为障碍物
    double b3 = 5.5;  // 向前能够移动的距离，目前用道路宽度代替
    double path_s = 0, path_r = 0;
    for (double s = 0.0; s < 5.0; s += 0.1){
        double r_fc = get_r_fc(s, b1, b2, b3);
        double r_sc = get_r_sc(s, b1, b2, b3);
        // std::cout << "r_fc: " << r_fc << std::endl;
        // std::cout << "r_sc: " << r_sc << std::endl;
        if(r_fc <= r_sc){
            double possible_r = r_fc;
            if (possible_r >= vehicle_param.vehicle_r_min){
                path_s = s;
                path_r = possible_r;
                // std::cout << "Planning Success!!!" << "path_s: " << path_s << "path_r: " << path_r << std::endl;
                break;
            }
        }
    }
    if (path_r < vehicle_param.vehicle_r_min){
        // 规划失败，返回空轨迹
        // std::cout << "Planning Fail!!!" <<  std::endl;
        return final_trajectory;
    }
    // 使用RS规划轨迹

    // 根据采样间隔生成轨迹，轨迹为从停车位到车，然后再reverse，后续改成直接正向生成
    // 直线部分
    for(double current_s = 0; current_s < path_s; current_s += trajectory_step_length){
        double current_point_x = parking_slot.center[0] + current_s * std::cos(parking_slot.theta);
        double current_point_y = parking_slot.center[1] + current_s * std::sin(parking_slot.theta);
        double current_point_theta = parking_slot.theta;
        final_trajectory.trajectory_points.emplace_back(Eigen::Vector3d(current_point_x, current_point_y, current_point_theta));
    }
    // 圆弧部分，有两个方向，选择末端模仿和车辆朝向最接近的方向
    double circle_start_point_x = parking_slot.center[0] + path_s * std::cos(parking_slot.theta);
    double circle_start_point_y = parking_slot.center[1] + path_s * std::sin(parking_slot.theta);
    // 判断是顺时针转还是逆时针转
    double clockwise_final_theta = parking_slot.theta - M_PI / 2.0;
    double counterclockwise_final_theta = parking_slot.theta + M_PI / 2.0;
    if (std::abs(normalize_angle(clockwise_final_theta - start_pose[2])) < std::abs(normalize_angle(counterclockwise_final_theta - start_pose[2]))){
        // 顺时针
        // std::cout << "clockwise theta is: " << parking_slot.theta <<std::endl;
        double start_angle = parking_slot.theta + M_PI / 2.0;
        double end_angle = parking_slot.theta;
        double delta_angle = -trajectory_step_length / path_r;
        double circle_center_x = circle_start_point_x + path_r * std::cos(clockwise_final_theta);
        double circle_center_y = circle_start_point_y + path_r * std::sin(clockwise_final_theta);
        for(double current_angle = start_angle; current_angle >= end_angle; current_angle += delta_angle){
            double current_point_x = circle_center_x + path_r * std::cos(current_angle);
            double current_point_y = circle_center_y + path_r * std::sin(current_angle);
            double current_point_theta = current_angle - M_PI / 2.0;
            final_trajectory.trajectory_points.emplace_back(Eigen::Vector3d(current_point_x, current_point_y, current_point_theta));
        }
    }
    else{
        // 逆时针
        // std::cout << "counterclockwisetheta is: " << parking_slot.theta << std::endl;
        double start_angle = parking_slot.theta - M_PI / 2.0;
        double end_angle = parking_slot.theta;
        // double start_angle = parking_slot.theta;
        // double end_angle = parking_slot.theta + M_PI / 2.0;
        double delta_angle = trajectory_step_length / path_r;
        double circle_center_x = circle_start_point_x + path_r * std::cos(counterclockwise_final_theta);
        double circle_center_y = circle_start_point_y + path_r * std::sin(counterclockwise_final_theta);
        for(double current_angle = start_angle; current_angle <= end_angle; current_angle += delta_angle){
            double current_point_x = circle_center_x + path_r * std::cos(current_angle);
            double current_point_y = circle_center_y + path_r * std::sin(current_angle);
            double current_point_theta = current_angle + M_PI / 2.0;
            final_trajectory.trajectory_points.emplace_back(Eigen::Vector3d(current_point_x, current_point_y, current_point_theta));
        }
    }
    // 拼接RS曲线部分

    return final_trajectory;
}

bool Planner::call_gcn_server(cv::Mat &avm_image, std::vector<std::pair<cv::Point, cv::Point>> &detected_point){
    detected_point.clear();
    cv::Mat image = avm_image.clone();
    int scale = 1;
    if (image.rows != target_input_size_.height && image.cols != target_input_size_.width) {
        cv::resize(image, image, target_input_size_);
        scale = 2;
    }

    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    parking_slot_detection::gcn_parking srv;
    
    
    srv.request.image_data = *image_msg;
    if (client_.call(srv)){
        for (size_t i = 0; i < srv.response.point0_x.size(); ++i) {
            int point0_x = srv.response.point0_x[i] * scale;
            int point0_y = srv.response.point0_y[i] * scale;
            int point1_x = srv.response.point1_x[i] * scale;
            int point1_y = srv.response.point1_y[i] * scale;
            int type = srv.response.types[i];
            detected_point.emplace_back(std::make_pair<cv::Point, cv::Point>(cv::Point(point0_x, point0_y), cv::Point(point1_x, point1_y)));
        }
        
    }
    else{
        std::cout << "Fail to call service" << std::endl;
        return false;
    }
    return true;
}

void Planner::avm_image_call_back(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        std::vector<std::pair<cv::Point, cv::Point>> detected_parking_points;
        if(!call_gcn_server(image, detected_parking_points)){
            std::cout << "call gcn server fail!!" << std::endl;
            return;
        }
        // 进行规划
        // std::cout << "start planning" << std::endl;
        for (auto parking_slot_points : detected_parking_points){
            ParkingSlot parking_slot = get_parking_slot(parking_slot_points);
            if (parking_slot.width < vehicle_param.vehicle_width + 0.2){
                continue;
            }
            Eigen::Vector3d vehicle_pose(0.0, 0.0, 0.0);
            Trajectory planned_trajectory = geometry_plan(vehicle_pose, parking_slot);
            
            // 进行可视化
            cv::Point point0 = parking_slot_points.first;
            cv::Point point1 = parking_slot_points.second;
            cv::circle(image, point0, 3, cv::Scalar(255, 0, 0), 2);
            cv::circle(image, point1, 3, cv::Scalar(0, 255, 0), 2);
            cv::Point parking_slot_center_img = point_front_vehicle_to_img(parking_slot.center);
            cv::circle(image, parking_slot_center_img, 3, cv::Scalar(0, 0, 255), 2);
            cv::Point test_point = point_front_vehicle_to_img(Eigen::Vector2d(5.0, 2.0));
            cv::circle(image, test_point, 3, cv::Scalar(0, 0, 0), 10, 10);
            if(planned_trajectory.trajectory_points.size() == 0){
                std::cout << "planning fail" << std::endl;
                continue;
            }
            std::cout << "parking_slot.theta: " << parking_slot.theta << std::endl;
            for (int i = 0; i < planned_trajectory.trajectory_points.size(); i+= 20){
                Eigen::Vector3d trajectory_point = planned_trajectory.trajectory_points[i];
                draw_rectangle(image, trajectory_point[0], trajectory_point[1], vehicle_param.vehicle_length, vehicle_param.vehicle_width, trajectory_point[2]);
            }
        }
        cv::imshow("detected_results", image);
        cv::waitKey(5);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Failed to convert ROS image message to OpenCV image: %s", e.what());
        return;
    }
}
