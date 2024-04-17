#ifndef PARKING_SLOT_MANAGER
#define PARKING_SLOT_MANAGER

#include <vector>
#include <string>

#include "common/common_struct.h"
#include "common/math_utils.h"

namespace APA{

    struct ParkingSlotManagerParam{
        int parking_slot_detected_threshold;
        int max_no_detected_cnt;
        double normal_parking_slot_length, normal_parking_slot_width;
    };

    class ParkingSlotManager{
        public:
            ParkingSlotManager();
            ParkingSlotManager(ParkingSlotManagerParam parking_slot_manager_param);
            bool get_parking_slot(std::pair<Eigen::Vector2d, Eigen::Vector2d> parking_slot_points, ParkingSlot &parking_slot);
            bool update(std::vector<ParkingSlot> detected_parking_slots); // 增加新车位

            std::vector<ParkingSlot> parking_slots;  // 经过多帧检测融合得到的车位，稳定车位
            std::vector<ParkingSlot> possible_parking_slots; // 暂定为可能的车位，未经过多帧融合检测

        private:
            bool check_similarity(ParkingSlot &parking_slot1, ParkingSlot &parking_slot2); // 判断是否为同一个车位
            bool update(ParkingSlot detected_parking_slot);
            
            int parking_slot_detected_threshold_; // 检测超过parking_slot_detected_threshold次数时认为为稳定车位
            int max_no_detected_cnt_; // 移除超过max_no_detected_cnt仍未被判定为稳定车位的不稳定车位
            double normal_parking_slot_length_, normal_parking_slot_width_; // 通常情况下的车位尺寸
    };
}


#endif