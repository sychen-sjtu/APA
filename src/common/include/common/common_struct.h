#ifndef COMMON_STRUCT
#define COMMON_STRUCT

#include <Eigen/Core>

namespace APA{

    struct VehicleParam{
        double K_f; // 前轴到车头的距离
        double K_r; // 后轴到车后的距离
        double L_f; // 前轴到中心的距离
        double L_r; // 后轴到中心的距离
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

        // 其它信息
        int detected_num;           // 检测到的次数，在检测的时候无需赋值
        int no_detected_cnt;        // 记录从车位创建开始有多少帧没有被检测到
        int id;                     // 车位的id
    };

}


#endif