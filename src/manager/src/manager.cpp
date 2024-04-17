#include "manager/manger.h"

namespace APA{

    Manager::Manager(ros::NodeHandle &nh){
        nh_ = nh;
        localization_init_ = false;

        #define PARAM(name, var)                                            \
        do {                                                                \
            if (!nh_.getParam(name, var)) {                                 \
                std::cout << "WARN!!! missing parameter " << name  << std::endl;     \
                return;                                                     \
            }                                                               \
            else{                                                           \
                std::cout << name << " : "<< var << std::endl;              \
            }                                                               \
        } while (0)

        // 读取参数
        int parking_slot_detected_threshold, max_no_detected_cnt;
        std::cout << "Start Getting Param: " << std::endl;
        PARAM("/parking_slot_detection/parking_slot_detected_threshold", parking_slot_detected_threshold);
        PARAM("/parking_slot_detection/max_no_detected_cnt", max_no_detected_cnt);

        nn_target_input_size_ = cv::Size(512, 512);
        avm_per_pixel_length_ = 0.01;
        avm_image_size_ = cv::Size(1024, 1024);
        avm_image_width_ = avm_image_size_.width * avm_per_pixel_length_;
        avm_image_height_ = avm_image_size_.height * avm_per_pixel_length_;


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

        // subscribe
        odom_sub_ = nh_.subscribe("/ekf_odom", 10, &Manager::odometryCallback, this);
        avm_image_sub_ = nh_.subscribe("/image_avm", 10, &Manager::avmImageCallback, this);
        // client
        parking_slot_detection_client_ = nh_.serviceClient<parking_slot_detection::gcn_parking>("/gcn_service");

        ParkingSlotManagerParam parking_slot_manager_param;
        parking_slot_manager_param.parking_slot_detected_threshold = parking_slot_detected_threshold;
        parking_slot_manager_param.max_no_detected_cnt = max_no_detected_cnt;
        parking_slot_manager_param.normal_parking_slot_length = 5.25;
        parking_slot_manager_param.normal_parking_slot_width = 2.5;

        parking_slot_manager = ParkingSlotManager(parking_slot_manager_param);
        visualizer = Visualizer(nh_);
        std::cout << "manager init finish" << std::endl;
        
    }

    Eigen::Vector2d Manager::point_front_img_to_vehicle(cv::Point img_point){
        double vehicle_point_x = avm_image_height_ / 2.0 - img_point.y * avm_per_pixel_length_;
        double vehicle_point_y = avm_image_width_ / 2.0 - img_point.x * avm_per_pixel_length_;
        return Eigen::Vector2d(vehicle_point_x, vehicle_point_y);
    }

    cv::Point Manager::point_front_vehicle_to_img(Eigen::Vector2d vehicle_point){
        int img_x = static_cast<int>((avm_image_width_ / 2.0 - vehicle_point[1]) / avm_per_pixel_length_);
        int img_y = static_cast<int>((avm_image_height_ / 2.0 - vehicle_point[0]) / avm_per_pixel_length_);
        return cv::Point(img_x, img_y);
    }

    bool Manager::point_front_vehicle_to_base(Eigen::Vector2d &vehicle_point, Eigen::Vector2d &base_point){
        base_point = vehicle_point + Eigen::Vector2d(vehicle_param.L_r, 0.0);
        return true;
    }

    bool Manager::point_front_img_to_base(cv::Point img_point, Eigen::Vector2d &base_point){
        double base_point_x = avm_image_height_ / 2.0 - img_point.y * avm_per_pixel_length_ + vehicle_param.L_r;
        double base_point_y = avm_image_width_ / 2.0 - img_point.x * avm_per_pixel_length_;
        base_point = Eigen::Vector2d(base_point_x, base_point_y);
        return true;
    }

    bool Manager::point_front_img_to_world(cv::Point img_point, Eigen::Vector2d &world_point){
        if (!localization_init_){
            return false;
        }
        Eigen::Vector2d base_point;
        point_front_img_to_base(img_point, base_point);
        world_point = ratation_from_base_to_world_ * base_point + self_position_;
        return true;
    }

    void Manager::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
        localization_init_ = true;
        
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        self_position_[0] = msg->pose.pose.position.x;
        self_position_[1] = msg->pose.pose.position.y;
        self_yaw_ = yaw;
        std::cout << "current yaw: " << self_yaw_ << std::endl;
        ratation_from_base_to_world_ << cos(self_yaw_), -sin(self_yaw_), sin(self_yaw_), cos(self_yaw_);
        
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        
        tf::Transform world_to_base_link = transform; // Define the transform from world to base_link

        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(world_to_base_link, ros::Time::now(), "world", "base_link"));
    }

    void Manager::avmImageCallback(const sensor_msgs::Image::ConstPtr& msg){
        if(!localization_init_){
            return;
        }
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::vector<std::pair<cv::Point, cv::Point>> image_detected_parking_points;
            if(!call_gcn_server(image, image_detected_parking_points)){
                std::cout << "call gcn server fail!!" << std::endl;
                return;
            }
            // 坐标系变换，到世界坐标系下
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> world_detected_parking_points(image_detected_parking_points.size());
            for(auto image_detected_parking_point : image_detected_parking_points){
                Eigen::Vector2d world_point_0, world_point_1;
                if(!point_front_img_to_world(image_detected_parking_point.first, world_point_0)){
                    std::cout << "warn no localization for point_front_img_to_world" << std::endl;
                    continue;
                }
                if(!point_front_img_to_world(image_detected_parking_point.second, world_point_1)){
                    std::cout << "warn no localization for point_front_img_to_world" << std::endl;
                    continue;
                }
                world_detected_parking_points.push_back(std::make_pair(world_point_0, world_point_1));
                std::cout << "world_point_0: " << world_point_0.transpose() << std::endl;
                std::cout << "world_point_1: " << world_point_1.transpose() << std::endl;
            }
            // 进行停车位管理
            std::cout << "detected parking slots: " << std::endl;
            // test data:
            // world_point_0: 12.0029 14.9928
            // world_point_1: 11.0591 13.9185
            // world_point_0: 9.68729 16.2121
            // world_point_1: 9.94374 16.8006
            ParkingSlot test_parking_slot;
            parking_slot_manager.get_parking_slot(std::make_pair(Eigen::Vector2d(9.68729, 16.2121), Eigen::Vector2d(9.94374, 16.8006)), test_parking_slot);
            std::cout << "test data: " << test_parking_slot.center.transpose() << " " << test_parking_slot.theta << std::endl;

            std::vector<ParkingSlot> possible_parking_slots;
            for(auto parking_slot_points : world_detected_parking_points){
                ParkingSlot parking_slot;
                if(!parking_slot_manager.get_parking_slot(parking_slot_points, parking_slot)){
                    continue;
                }
                std::cout << parking_slot.center.transpose() << " " << parking_slot.theta << std::endl;
                possible_parking_slots.push_back(parking_slot);
            }
            parking_slot_manager.update(possible_parking_slots);
            // 对车位进行可视化
            std::cout << "all possible parking slots" << std::endl;
            for(auto parking_slot : parking_slot_manager.possible_parking_slots){
                std::cout << parking_slot.center.transpose() << " " << parking_slot.theta << std::endl;
                possible_parking_slots.push_back(parking_slot);
            }
            visualizer.publish_parking_slots(parking_slot_manager.possible_parking_slots);
            
            // // 获取车位并进行规划
            // std::cout << "start planning" << std::endl;
            // for (auto parking_slot_points : image_detected_parking_points){
            //     cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
            //     ParkingSlot parking_slot = get_parking_slot(parking_slot_points);
            //     if (parking_slot.width < vehicle_param.vehicle_width + 0.2){
            //         continue;
            //     }
            //     Eigen::Vector3d vehicle_pose(0.0, 0.0, 0.0);
            //     Trajectory planned_trajectory = geometry_plan(vehicle_pose, parking_slot);
            //     // 在这里做检测，主要是容易可视化
            //     std::vector<boost_polygon> obstacles_list = get_obstacles_from_parking_slot(parking_slot, 5.5, 10);
            //     // 进行可视化
            //     cv::Point point0 = parking_slot_points.first;
            //     cv::Point point1 = parking_slot_points.second;
            //     cv::circle(image, point0, 3, cv::Scalar(255, 0, 0), 2);
            //     cv::circle(image, point1, 3, cv::Scalar(0, 255, 0), 2);
            //     cv::Point parking_slot_center_img = point_front_vehicle_to_img(parking_slot.center);
            //     cv::circle(image, parking_slot_center_img, 3, cv::Scalar(0, 0, 255), 2);
            //     if(planned_trajectory.trajectory_points.size() == 0){
            //         std::cout << "planning fail" << std::endl;
            //         continue;
            //     }
            //     std::cout << "parking_slot.theta: " << parking_slot.theta << std::endl;
            //     // for (int i = 0; i < planned_trajectory.trajectory_points.size(); i+= 50){
            //     //     Eigen::Vector3d trajectory_point = planned_trajectory.trajectory_points[i];
            //     //     double current_theta = trajectory_point[2];
            //     //     draw_rectangle(image, trajectory_point[0] + vehicle_param.L_r * std::cos(current_theta), trajectory_point[1] + vehicle_param.L_r * std::sin(current_theta), vehicle_param.vehicle_length, vehicle_param.vehicle_width, trajectory_point[2]);
            //     //     cv::putText(image, std::to_string(i), cv::Point(point_front_vehicle_to_img(Eigen::Vector2d(trajectory_point[0], trajectory_point[1]))), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            //     // }
            //     for(int i = 0; i < planned_trajectory.trajectory_points.size() - 10; i += 10){
            //         Eigen::Vector3d current_trajectory_point = planned_trajectory.trajectory_points[i];
            //         Eigen::Vector3d next_trajectory_point = planned_trajectory.trajectory_points[i + 10];
            //         cv::Point current_trajectory_point_image = point_front_vehicle_to_img(Eigen::Vector2d(current_trajectory_point[0], current_trajectory_point[1]));
            //         cv::Point next_trajectory_point_image = point_front_vehicle_to_img(Eigen::Vector2d(next_trajectory_point[0], next_trajectory_point[1]));
            //         cv::line(image, current_trajectory_point_image, next_trajectory_point_image, color, 2);
            //         cv::putText(image, std::to_string(i), current_trajectory_point_image, cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 0.5);

            //         // // 判断轨迹是否发生碰撞
            //         // double current_theta = current_trajectory_point[2];
            //         // double vehicle_center_x = current_trajectory_point[0] + vehicle_param.L_r * std::cos(current_theta);
            //         // double vehicle_center_y = current_trajectory_point[1] + vehicle_param.L_r * std::sin(current_theta);
            //         // if (polygon_validate_check(Eigen::Vector3d(vehicle_center_x, vehicle_center_y, current_theta), obstacles_list)){
            //         //     draw_rectangle(image, vehicle_center_x, vehicle_center_y, vehicle_param.vehicle_length, vehicle_param.vehicle_width, current_theta, color);
            //         // }

            //     }
            // }
            // cv::imshow("detected_results", image);
            // // if(if_save_results){
            // //     std::stringstream ss;
            // //     ss << std::setw(5) << std::setfill('0') << frame_cnt;
            // //     std::string idStr = ss.str();
            // //     std::string output_file = output_dir + idStr + ".png";
            // //     cv::imwrite(output_file, image);
            // // }
            // cv::waitKey(2);
            avm_image_frame_cnt++;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Failed to convert ROS image message to OpenCV image: %s", e.what());
            return;
        }
    }

    bool Manager::call_gcn_server(cv::Mat &avm_image, std::vector<std::pair<cv::Point, cv::Point>> &detected_point){
        detected_point.clear();
        cv::Mat image = avm_image.clone();
        double scale = 1.0;
        if (image.rows != nn_target_input_size_.height && image.cols != nn_target_input_size_.width) {
            cv::resize(image, image, nn_target_input_size_);
            scale = (double)nn_target_input_size_.height / (double)image.rows;
        }
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        parking_slot_detection::gcn_parking srv;
        
        srv.request.image_data = *image_msg;
        if (parking_slot_detection_client_.call(srv)){
            for (size_t i = 0; i < srv.response.point0_x.size(); ++i) {
                int point0_x_image = srv.response.point0_x[i] * scale;
                int point0_y_image = srv.response.point0_y[i] * scale;
                int point1_x_image = srv.response.point1_x[i] * scale;
                int point1_y_image = srv.response.point1_y[i] * scale;
                int type = srv.response.types[i];
                detected_point.emplace_back(std::make_pair<cv::Point, cv::Point>(cv::Point(point0_x_image, point0_y_image), cv::Point(point1_x_image, point1_y_image)));
            }
        }
        else{
            std::cout << "Fail to call service" << std::endl;
            return false;
        }
        return true;
    }
}