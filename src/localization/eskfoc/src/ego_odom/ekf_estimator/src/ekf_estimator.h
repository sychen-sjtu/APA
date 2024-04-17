//
// Created by ubuntu on 24-2-26.
//

#ifndef DATA_GENERATOR_EKF_ESTIMATOR_H
#define DATA_GENERATOR_EKF_ESTIMATOR_H

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cyber_msgs/SpeedFeedback.h>
#include <sensor_msgs/NavSatFix.h>
#include "utility.h"
#include <queue>
#include <thread>
#include <mutex>

#include <diankong/VehicleFeedback.h>

#define IF_USE_GPS 0
#define IF_USE_WHL 1

// typedef nav_msgs::OdometryConstPtr WheelDataPtr;
// typedef geometry_msgs::PoseStampedConstPtr GNSSDataPtr;
// typedef cyber_msgs::SpeedFeedbackConstPtr WheelDataPtr;
// typedef sensor_msgs::NavSatFixConstPtr GNSSDataPtr;

struct IMUData{
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
using IMUDataPtr = std::shared_ptr<IMUData>;

struct GNSSData
{
    double timestamp;

    Eigen::Vector3d lla;
    Eigen::Matrix3d cov;
};
using GNSSDataPtr = std::shared_ptr<GNSSData>;

struct WHLData
{
    double timestamp;

    Eigen::Vector3d speed;
};

using WheelDataPtr = std::shared_ptr<WHLData>;

enum StateIndex : uint {
    R = 0,                   // (3 dimention) rotation in world frame
    P = 3,                   // (3 dimention) position in world frame
    V = 6,                   // (3 dimention) velocity in world frame
    BA = 9,                 // (3 dimention) IMU acceleration bias
    BG = 12,                 // (3 dimention) IMU gyroscope bias
    IMU_INSTALL_ANGLE = 15,  // (3 dimention) imu install error IMU_INSTALL_ANGLE
    WS = 18,                 // (1 dimention) wheel speed ratio
    STATE_TOTAL = 19
};
    // IMU_INSTALL_ANGLE = 15,  // (3 dimention) imu install error imu_q_veh
    // WS = 18,                 // (1 dimention) wheel speed ratio

enum StateNoiseIndex : uint {
    ACC_NOISE = 0,         // linear acceleration change
    GYRO_NOISE = 3,          // angular velocity change
    ACC_RANDOM_WALK = 6,   // IMU aceleration bias random walk
    GYRO_RANDOM_WALK = 9,  // IMU gyroscope bias random walk
    NOISE_TOTAL = 12
};

struct State {
    Eigen::Matrix3d R_q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    Eigen::Matrix3d R_imu;
    long double time;
    double ws;
    Eigen::MatrixXd P;
};
using StatePtr = std::shared_ptr<State>;

constexpr double D_R = M_PI / 180.;
constexpr double R_D = 180. / M_PI;

class EkfEstimator {
public:
    EkfEstimator(ros::NodeHandle &nh);
    ~EkfEstimator();
    void RosNodeRegistration(ros::NodeHandle &n);

private:
    bool InitState();
    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
    // void WheelCallback(const cyber_msgs::SpeedFeedbackConstPtr &wheel_msg);
    void WheelCallback(const diankong::VehicleFeedbackConstPtr &wheel_msg);
    void GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg) ;

    bool process_IMU_Data(IMUDataPtr imu_data_ptr);
    void PredictByImu(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr);
    void UpdateByWheel(WheelDataPtr wheel_data_ptr);
    void UpdateByGps(GNSSDataPtr gnss_data_ptr);
    void Publish();
    ros::Subscriber sub_imu_, sub_wheel_, sub_gps_;

    State state_;
    nav_msgs::Path path_;
    ros::Publisher pub_path_, pub_odometry_,gnss_path_pub_;
    Eigen::Vector3d gravity_;
    bool stateInit_ = false;
    long double init_time_ = 0.;
    
    Eigen::MatrixXd Q_; // variance matrix of imu
    Eigen::MatrixXd whl_Rm_; // variance matrix of wheel odometer
    Eigen::MatrixXd gps_Rm_; // variance matrix of gps

    bool shutdown_ = false;
    std::deque<IMUDataPtr> imu_buf_;
    IMUDataPtr last_imu_ptr_;
    bool parameter_lock = true;
    const double IMU_Std = 3.0;
    const int IMU_BUF_SIZE = 100;
    Eigen::Vector3d p_I_GNSS_;
    Eigen::Vector3d init_lla_;
    nav_msgs::Path gnss_path_;

    // log files
    std::ofstream file_gt_xyz;
    std::ofstream file_our_state_;
    std::ofstream file_our_lla;
    std::ofstream file_gt_lla;
    
    double acc_n_var, gyr_n_var, acc_rw_var, gyr_rw_var,imu_mount_rw_var;
    double whl_odom_var,gnss_noise_var;
    // double ACC_NOISE_VAR = 1e-3;
    // double GYRO_NOISE_VAR = 1e-4;
    // double ACC_RANDOM_WALK_VAR = 1e-7;
    // double GYRO_RANDOM_WALK_VAR = 1e-8;
    // double WHEEL_ODOMETER_VAR = 1;
    // double GNSS_NOISE_VAR = 1e-2;

    bool IF_GPS_DENY = false;
    bool IF_FILE_OUT = false;
};


#endif //DATA_GENERATOR_EKF_ESTIMATOR_H
