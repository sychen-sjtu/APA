#include "planning/planner.h"


Planner::Planner(ros::NodeHandle &nh){
    nh_ = nh;
    
    #define PARAM(name, var)                                \
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
    PARAM("/planning_node/output_dir", output_dir);
    PARAM("/planning_node/save_results", if_save_results);
    
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
    
    shotptr =std::make_shared<ompl::base::ReedsSheppStateSpace>(vehicle_param.vehicle_r_min);

    cv::namedWindow("detected_results");
}

Planner::~Planner(){
    cv::destroyAllWindows();
}

void Planner::draw_rectangle(cv::Mat &image, double center_x, double center_y, double length, double width, double theta, cv::Scalar color){
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
        cv::line(image, rectangle_vertices_image[i % point_num], rectangle_vertices_image[(i + 1) % point_num], color, 2);
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

std::vector<Planner::boost_polygon> Planner::get_obstacles_from_parking_slot(ParkingSlot &parking_slot, double road_width, double min_obstacle_length){
    // 将停车位和道路以外的区域全部看做障碍物（最保守的做法），有三块障碍物：车位左右，车位对面
    std::vector<Planner::boost_polygon> obstacle_list;
    double parking_slot_center_x = parking_slot.center[0];
    double parking_slot_center_y = parking_slot.center[1];
    double parking_slot_theta = parking_slot.theta;
    double half_length = parking_slot.length / 2.0;
    double half_width = parking_slot.length / 2.0;

    Eigen::Vector2d parking_slot_direction_vector(std::cos(parking_slot_theta), std::sin(parking_slot_theta));

    // 车辆左右的障碍物，从车位中心，先计算某一边的两个端点，再进行延长
    for(double obstacle_theta : {parking_slot_theta  - M_PI/2.0, parking_slot_theta + M_PI/2.0}){
        Eigen::Vector2d parking_slot_lateral_direction_vector(std::cos(obstacle_theta), std::sin(obstacle_theta));
        
        // 车位前方的一个顶点
        Eigen::Vector2d point1 = parking_slot.center + half_length * parking_slot_direction_vector + half_width * parking_slot_lateral_direction_vector;
        
        // 车位后方的一个顶点
        Eigen::Vector2d point2 = parking_slot.center - half_length * parking_slot_direction_vector + half_width * parking_slot_lateral_direction_vector;
        
        // 车位前方的一个顶点沿车位横向延长
        Eigen::Vector2d point3 = point1 + min_obstacle_length * parking_slot_lateral_direction_vector;

        // 车位后方的一个顶点沿车位横向延长
        Eigen::Vector2d point4 = point2 + min_obstacle_length * parking_slot_lateral_direction_vector;

        // 构造障碍物矩形
        std::vector<Planner::boost_point> points = {
            {point1[0], point1[1]}, 
            {point2[0], point2[1]},
            {point3[0], point3[1]},
            {point4[0], point4[1]}
        };
        Planner::boost_polygon obstacle_poly;
        boost::geometry::append(obstacle_poly, points);
        boost::geometry::correct(obstacle_poly);
        obstacle_list.emplace_back(obstacle_poly);
    }
    // 车位对面的障碍物
    {
        Eigen::Vector2d obstacle_near_line_center = parking_slot.center + (half_length + road_width) * parking_slot_direction_vector;
        Eigen::Vector2d obstacle_far_line_center = parking_slot.center + (half_length + road_width + min_obstacle_length) * parking_slot_direction_vector;
        Eigen::Vector2d parking_slot_lateral_direction_vector_1(std::cos(parking_slot_theta  - M_PI/2.0), std::sin(parking_slot_theta  - M_PI/2.0));
        Eigen::Vector2d parking_slot_lateral_direction_vector_2(std::cos(parking_slot_theta  + M_PI/2.0), std::sin(parking_slot_theta  + M_PI/2.0));
        Eigen::Vector2d shift_vector_1 = min_obstacle_length * parking_slot_lateral_direction_vector_1;
        Eigen::Vector2d shift_vector_2 = min_obstacle_length * parking_slot_lateral_direction_vector_2;

        Eigen::Vector2d point1 = obstacle_near_line_center + shift_vector_1;
        Eigen::Vector2d point2 = obstacle_near_line_center + shift_vector_2;
        Eigen::Vector2d point3 = obstacle_far_line_center + shift_vector_2;
        Eigen::Vector2d point4 = obstacle_far_line_center + shift_vector_1;

        // 构造障碍物矩形
        std::vector<Planner::boost_point> points = {
            {point1[0], point1[1]}, 
            {point2[0], point2[1]},
            {point3[0], point3[1]},
            {point4[0], point4[1]}
        };
        Planner::boost_polygon obstacle_poly;
        boost::geometry::append(obstacle_poly, points);
        boost::geometry::correct(obstacle_poly);
        obstacle_list.emplace_back(obstacle_poly);
    }

    return obstacle_list;
}

// 输入车辆中心坐标与障碍物vector
bool Planner::polygon_validate_check(Eigen::Vector3d pose, std::vector<boost_polygon> & obstacle_list){
    // 构建自车的polygon
    // 计算矩形的旋转角度对应的正弦值和余弦值
    double sin_val = std::sin(pose[2]);
    double cos_val = std::cos(pose[2]);
    double center_x = pose[0];
    double center_y = pose[1];
    double length = vehicle_param.vehicle_length;
    double width = vehicle_param.vehicle_width;
    // 计算矩形的四个顶点坐标
    double x1 = center_x - length / 2 * cos_val - width / 2 * sin_val;
    double y1 = center_y - length / 2 * sin_val + width / 2 * cos_val;

    double x2 = center_x + length / 2 * cos_val - width / 2 * sin_val;
    double y2 = center_y + length / 2 * sin_val + width / 2 * cos_val;

    double x3 = center_x + length / 2 * cos_val + width / 2 * sin_val;
    double y3 = center_y + length / 2 * sin_val - width / 2 * cos_val;

    double x4 = center_x - length / 2 * cos_val + width / 2 * sin_val;
    double y4 = center_y - length / 2 * sin_val - width / 2 * cos_val;

    std::vector<Planner::boost_point> vehicle_points = {
        {x1, y1}, 
        {x2, y2},
        {x3, y3},
        {x4, y4}
    };
    Planner::boost_polygon vehicle_poly;
    boost::geometry::append(vehicle_poly, vehicle_points);
    boost::geometry::correct(vehicle_poly);

    for(auto obstacle : obstacle_list){
        if (boost::geometry::intersects(obstacle, vehicle_poly)){
            return true;
        }
    }
    return false;


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

    // 先生成后半部分的轨迹，然后再进行RS规划 
    double parking_end_point_x = parking_slot.center[0] - vehicle_param.L_r * std::cos(parking_slot.theta);
    double parking_end_point_y = parking_slot.center[1] - vehicle_param.L_r * std::sin(parking_slot.theta);
    // 这个规划方式得到的路径车中心的规划路径，这里转化为车辆后轴中心试试
    // 圆弧部分，有两个方向，选择末端模仿和车辆朝向最接近的方向

    double circle_start_point_x = parking_end_point_x + path_s * std::cos(parking_slot.theta); // 从运动角度看，这是圆弧轨迹的终点
    double circle_start_point_y = parking_end_point_y + path_s * std::sin(parking_slot.theta);
    // 判断是顺时针转还是逆时针转，这里的顺时针和逆时针是，圆的起始角度，终止角度，都是从离开车位来说的，因此在加入轨迹时需要倒序加入
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
        // for(double current_angle = start_angle; current_angle >= end_angle; current_angle += delta_angle){
        for(double current_angle = end_angle; current_angle <= start_angle; current_angle -= delta_angle){
            double current_point_theta = current_angle - M_PI / 2.0;
            double current_point_x = circle_center_x + path_r * std::cos(current_angle);
            double current_point_y = circle_center_y + path_r * std::sin(current_angle);
            Eigen::Vector3d current_pose(current_point_x, current_point_y, current_point_theta);
            final_trajectory.trajectory_points.emplace_back(current_pose);
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
        // for(double current_angle = start_angle; current_angle <= end_angle; current_angle += delta_angle){
        for(double current_angle = end_angle; current_angle >= start_angle; current_angle -= delta_angle){
            double current_point_theta = current_angle + M_PI / 2.0;
            double current_point_x = circle_center_x + path_r * std::cos(current_angle);
            double current_point_y = circle_center_y + path_r * std::sin(current_angle);
            Eigen::Vector3d current_pose(current_point_x, current_point_y, current_point_theta);
            final_trajectory.trajectory_points.emplace_back(current_pose);
        }
    }

    // 根据采样间隔生成轨迹，圆弧终点到停车位
    // 直线部分
    for(double current_s = path_s; current_s >= 0.0; current_s -= trajectory_step_length){
        double current_point_x = parking_end_point_x + current_s * std::cos(parking_slot.theta);
        double current_point_y = parking_end_point_y + current_s * std::sin(parking_slot.theta);
        double current_point_theta = parking_slot.theta;
        Eigen::Vector3d current_pose(current_point_x, current_point_y, current_point_theta);
        final_trajectory.trajectory_points.emplace_back(current_pose);
    }

    // 使用RS规划轨迹
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    from[0] = start_pose[0] - vehicle_param.L_r * std::cos(start_pose[2]);
    from[1] = start_pose[1] - vehicle_param.L_r * std::sin(start_pose[2]);
    from[2] = start_pose[2];
    // RS规划的终点是几何方法规划的起点
    to[0] = final_trajectory.trajectory_points[0][0];
    to[1] = final_trajectory.trajectory_points[0][1];
    to[2] = final_trajectory.trajectory_points[0][2];
    std::vector<double> reals;
    double len = shotptr->distance(from(), to());
    std::vector<Eigen::Vector3d> rs_trajectory_points;
    for (double l = 0.0; l <=len; l += trajectory_step_length)
    {
        shotptr->interpolate(from(), to(), l/len, s());
        reals = s.reals();
        Eigen::Vector3d current_pose = Eigen::Vector3d(reals[0], reals[1], reals[2]);
        rs_trajectory_points.emplace_back(current_pose);
        // AINFO << "( " << reals[0] << ", " <<  reals[1] << ", " << reals[2] << ")";
    }

    // 合并轨迹
    final_trajectory.trajectory_points.insert(final_trajectory.trajectory_points.begin(), 
                                            std::make_move_iterator(rs_trajectory_points.begin()), 
                                            std::make_move_iterator(rs_trajectory_points.end()));
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
        std::cout << "start planning" << std::endl;
        for (auto parking_slot_points : detected_parking_points){
            cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
            ParkingSlot parking_slot = get_parking_slot(parking_slot_points);
            if (parking_slot.width < vehicle_param.vehicle_width + 0.2){
                continue;
            }
            Eigen::Vector3d vehicle_pose(0.0, 0.0, 0.0);
            Trajectory planned_trajectory = geometry_plan(vehicle_pose, parking_slot);
            // 在这里做检测，主要是容易可视化
            std::vector<boost_polygon> obstacles_list = get_obstacles_from_parking_slot(parking_slot, 5.5, 10);
            // 进行可视化
            cv::Point point0 = parking_slot_points.first;
            cv::Point point1 = parking_slot_points.second;
            cv::circle(image, point0, 3, cv::Scalar(255, 0, 0), 2);
            cv::circle(image, point1, 3, cv::Scalar(0, 255, 0), 2);
            cv::Point parking_slot_center_img = point_front_vehicle_to_img(parking_slot.center);
            cv::circle(image, parking_slot_center_img, 3, cv::Scalar(0, 0, 255), 2);
            if(planned_trajectory.trajectory_points.size() == 0){
                std::cout << "planning fail" << std::endl;
                continue;
            }
            std::cout << "parking_slot.theta: " << parking_slot.theta << std::endl;
            // for (int i = 0; i < planned_trajectory.trajectory_points.size(); i+= 50){
            //     Eigen::Vector3d trajectory_point = planned_trajectory.trajectory_points[i];
            //     double current_theta = trajectory_point[2];
            //     draw_rectangle(image, trajectory_point[0] + vehicle_param.L_r * std::cos(current_theta), trajectory_point[1] + vehicle_param.L_r * std::sin(current_theta), vehicle_param.vehicle_length, vehicle_param.vehicle_width, trajectory_point[2]);
            //     cv::putText(image, std::to_string(i), cv::Point(point_front_vehicle_to_img(Eigen::Vector2d(trajectory_point[0], trajectory_point[1]))), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            // }
            for(int i = 0; i < planned_trajectory.trajectory_points.size() - 10; i += 10){
                Eigen::Vector3d current_trajectory_point = planned_trajectory.trajectory_points[i];
                Eigen::Vector3d next_trajectory_point = planned_trajectory.trajectory_points[i + 10];
                cv::Point current_trajectory_point_image = point_front_vehicle_to_img(Eigen::Vector2d(current_trajectory_point[0], current_trajectory_point[1]));
                cv::Point next_trajectory_point_image = point_front_vehicle_to_img(Eigen::Vector2d(next_trajectory_point[0], next_trajectory_point[1]));
                cv::line(image, current_trajectory_point_image, next_trajectory_point_image, color, 2);
                cv::putText(image, std::to_string(i), current_trajectory_point_image, cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 0.5);

                // 判断轨迹是否发生碰撞
                double current_theta = current_trajectory_point[2];
                double vehicle_center_x = current_trajectory_point[0] + vehicle_param.L_r * std::cos(current_theta);
                double vehicle_center_y = current_trajectory_point[1] + vehicle_param.L_r * std::sin(current_theta);
                if (polygon_validate_check(Eigen::Vector3d(vehicle_center_x, vehicle_center_y, current_theta), obstacles_list)){
                    draw_rectangle(image, vehicle_center_x, vehicle_center_y, vehicle_param.vehicle_length, vehicle_param.vehicle_width, current_theta, color);
                }

            }
        }
        cv::imshow("detected_results", image);
        if(if_save_results){
            std::stringstream ss;
            ss << std::setw(5) << std::setfill('0') << frame_cnt;
            std::string idStr = ss.str();
            std::string output_file = output_dir + idStr + ".png";
            cv::imwrite(output_file, image);
        }
        cv::waitKey(2);
        frame_cnt++;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Failed to convert ROS image message to OpenCV image: %s", e.what());
        return;
    }
}
