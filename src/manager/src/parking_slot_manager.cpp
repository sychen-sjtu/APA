#include "manager/parking_slot_manager.h"

namespace APA{
    
    ParkingSlotManager::ParkingSlotManager(){}

    ParkingSlotManager::ParkingSlotManager(ParkingSlotManagerParam parking_slot_manager_param){
        parking_slot_detected_threshold_ = parking_slot_manager_param.parking_slot_detected_threshold;
        max_no_detected_cnt_= parking_slot_manager_param.max_no_detected_cnt;
        normal_parking_slot_length_ = parking_slot_manager_param.normal_parking_slot_length;
        normal_parking_slot_width_ = parking_slot_manager_param.normal_parking_slot_width;

        parking_slots.clear();
        possible_parking_slots.clear();
    }

    bool ParkingSlotManager::update(std::vector<ParkingSlot> detected_parking_slots){
        for(auto detected_parking_slot : detected_parking_slots){
            if(!update(detected_parking_slot)){
                return false;
            }
        }

        // 移除不稳定车位中长时间没检测到的车位，这些车位可能由于误检测导致
        auto it = possible_parking_slots.begin();
        while (it != possible_parking_slots.end()) {
            it->no_detected_cnt++;
            if(it->no_detected_cnt > max_no_detected_cnt_){
                it = possible_parking_slots.erase(it);
            }
            else{
                it++;
            }
        }
    }

    bool ParkingSlotManager::update(ParkingSlot detected_parking_slot){
        // 首先判断这个车位是否是稳定车位
        for(auto parking_slot : parking_slots){
            if(check_similarity(detected_parking_slot, parking_slot)){
                parking_slot.detected_num++;
                return true;
            }
        }
        // 判断这个车位是否在不稳定车位中
        auto it = possible_parking_slots.begin();
        while (it != possible_parking_slots.end()) {
            if(check_similarity(detected_parking_slot, *it)){
                it->detected_num++;
                if(it->detected_num > parking_slot_detected_threshold_){
                    parking_slots.push_back(*it);
                    parking_slots.back().id = parking_slots.size() - 1;
                    it = possible_parking_slots.erase(it);
                    return true;
                }
                break;
            }else{
                it++;
            }
        }
        // 这个车位不在不稳定车位中，把他加入不稳定车位
        possible_parking_slots.emplace_back(detected_parking_slot);
        possible_parking_slots.back().detected_num = 0;
        possible_parking_slots.back().no_detected_cnt = 0;

        return true;
    }


    bool ParkingSlotManager::check_similarity(ParkingSlot &parking_slot1, ParkingSlot &parking_slot2){
        double delta_dist = (parking_slot1.center - parking_slot2.center).norm();
        double delta_angle = std::abs(normalize_angle(parking_slot1.theta - parking_slot2.theta));
        if (delta_dist < 1.0 && delta_angle < M_PI / 6.0){
            return true;
        }
        else{
            return false;
        }
    }

    // 传入world坐标系下的点，获得车位信息
    bool ParkingSlotManager::get_parking_slot(std::pair<Eigen::Vector2d, Eigen::Vector2d> parking_slot_points, ParkingSlot &parking_slot){
        Eigen::Vector2d point0 = parking_slot_points.first;
        Eigen::Vector2d point1 = parking_slot_points.second;
        Eigen::Vector2d point0_point1_vec = point1 - point0;
        if(std::abs(point0_point1_vec.norm() - normal_parking_slot_width_) > 0.7){
            // 车位宽度不匹配，直接过滤
            return false;
        }
        Eigen::Vector2d point0_point1_vec_normalized = point0_point1_vec.normalized();
        Eigen::Vector2d in_line_center = (point0 + point1) / 2.0;

        double parking_slot_theta = std::atan2(-point0_point1_vec_normalized[0], point0_point1_vec_normalized[1]); // 与二者连线垂直
        double parking_slot_center_x = in_line_center[0] - std::cos(parking_slot_theta) * normal_parking_slot_length_ / 2.0;
        double parking_slot_center_y = in_line_center[1] - std::sin(parking_slot_theta) * normal_parking_slot_length_ / 2.0;
        Eigen::Vector2d parking_slot_center(parking_slot_center_x, parking_slot_center_y);
        
        parking_slot.center = parking_slot_center;
        parking_slot.theta  = parking_slot_theta;
        parking_slot.length = normal_parking_slot_length_;
        parking_slot.width = point0_point1_vec.norm();

        return true;
    }
}