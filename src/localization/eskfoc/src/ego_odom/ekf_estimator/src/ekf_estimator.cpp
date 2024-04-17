//
// Created by ubuntu on 24-2-26.
//

#include "ekf_estimator.h"

// #define IF_GPS_DENY 1
#define IF_WS_ERROR 0
#define IF_IMU_MOUNT_ERROR 0

// 三段gps不可用的时间区间 50-80  120-160  200-250
std::vector<Eigen::Vector2d> deny_time = {Eigen::Vector2d(80, 100), Eigen::Vector2d(120, 190), Eigen::Vector2d(220, 240)};
// std::vector<Eigen::Vector2d> deny_time = {Eigen::Vector2d(100, 120), Eigen::Vector2d(140, 210), Eigen::Vector2d(240, 260)};
// std::vector<Eigen::Vector2d> deny_time =  {Eigen::Vector2d(800, 2000)};

EkfEstimator::EkfEstimator(ros::NodeHandle &nh)
{
    ROS_WARN("loading parameter");

    nh.param<double>("acc_noise", acc_n_var, 1e-2);
    nh.param<double>("gyr_noise", gyr_n_var, 1e-3);
    nh.param<double>("acc_bias_noise", acc_rw_var, 1e-7);
    nh.param<double>("gyr_bias_noise", gyr_rw_var, 1e-8);
    nh.param<double>("imu_mount_rw_var", imu_mount_rw_var, 1e-6);
    nh.param<double>("whl_odom_var", whl_odom_var, 100);
    nh.param<double>("gnss_noise_var", gnss_noise_var, 1e-2);

    nh.param<bool>("IF_GPS_DENY", IF_GPS_DENY, false);
    nh.param<bool>("IF_FILE_OUT", IF_FILE_OUT, false);

    ROS_WARN("acc_noise, gyr_noise, acc_bias_noise, gyr_bias_noise, whl_odom_var, gnss_noise_var: %f, %f, %f, %f, %f, %f", acc_n_var, gyr_n_var, acc_rw_var, gyr_rw_var, whl_odom_var, gnss_noise_var);
    ROS_WARN("IF_GPS_DENY: %d", IF_GPS_DENY); 
    ROS_WARN("IF_FILE_OUT: %d", IF_FILE_OUT);

    // ekf_ptr_ = std::make_unique<EKF>(acc_n, gyr_n, acc_w, gyr_w);

    Eigen::Matrix3d R_I = Eigen::Matrix3d::Identity();
    state_.R_q = R_I;
    state_.p.setZero();
    state_.v.setZero();
    state_.ba.setZero();
    state_.bg.setZero();
    state_.R_imu = R_I;
    state_.ws = 1.0;

    state_.P = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);
    state_.P.block<3, 3>(StateIndex::P, StateIndex::P) = 100. * Eigen::Matrix3d::Identity();
    state_.P(StateIndex::P + 2, StateIndex::P + 2) = 100;
    state_.P.block<3, 3>(StateIndex::V, StateIndex::V) = 100. * Eigen::Matrix3d::Identity();
    state_.P(StateIndex::V + 2, StateIndex::V + 2) = 100;
    state_.P.block<3, 3>(StateIndex::R, StateIndex::R) = 100. * Eigen::Matrix3d::Identity();
    state_.P(StateIndex::R+2, StateIndex::R+2) = 900.;
    state_.P.block<3, 3>(StateIndex::BA, StateIndex::BA) = 0.0004 * Eigen::Matrix3d::Identity();
    state_.P.block<3, 3>(StateIndex::BG, StateIndex::BG) = 0.0004 * Eigen::Matrix3d::Identity();
    state_.P.block<3, 3>(StateIndex::IMU_INSTALL_ANGLE, StateIndex::IMU_INSTALL_ANGLE) = imu_mount_rw_var * Eigen::Matrix3d::Identity();
    // state_.P(StateIndex::IMU_INSTALL_ANGLE+2, StateIndex::IMU_INSTALL_ANGLE+2) = 1e-6; //限制yaw
    state_.P(StateIndex::WS, StateIndex::WS) = 1e-5;
    gravity_ = Eigen::Vector3d(0, 0, -9.73);
    Q_ = Eigen::Matrix<double, StateNoiseIndex::NOISE_TOTAL, StateNoiseIndex::NOISE_TOTAL>::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_NOISE, StateNoiseIndex::ACC_NOISE) =
        acc_n_var * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_NOISE, StateNoiseIndex::GYRO_NOISE) =
        gyr_n_var * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_RANDOM_WALK, StateNoiseIndex::ACC_RANDOM_WALK) =
        acc_rw_var * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK) =
        gyr_rw_var * Eigen::Matrix3d::Identity();
    whl_Rm_ = Eigen::Matrix3d::Identity() * whl_odom_var;
    gps_Rm_ = Eigen::Matrix3d::Identity() * gnss_noise_var;

    if(IF_FILE_OUT)
    {
    // log files
    // file_gt_lla.open("/home/heron/Desktop/our_gt_lla.txt"); // gt xyz
    // file_our_state_.open("/home/heron/Desktop/our_fused_state.csv");
    // file_our_lla.open("/home/heron/Desktop/our_state_lla.txt"); // our lla
    // file_gt_xyz.open("/home/heron/Desktop/our_gt_xyz.csv");     // gt lla
    }

    ROS_WARN("parameter load finish");

    // main_thread_ = std::thread(&EkfEstimator::MainProcessThread, this);
}

EkfEstimator::~EkfEstimator()
{
    file_gt_xyz.close();
    file_our_state_.close();
    file_our_lla.close();
    file_gt_lla.close();
    // shutdown_ = true;
    // main_thread_.join();
}

bool EkfEstimator::InitState()
{
    ROS_ERROR("init state");

    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (auto imu_data : imu_buf_)
    {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_.size();
    std::cout << "[ ESKF ] Mean acc: " << mean_acc[0] << " " << mean_acc[1] << " " << mean_acc[2] << std::endl;

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (auto imu_data : imu_buf_)
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf_.size()).cwiseSqrt();
    std::cout << "[ ESKF ] Std acc : " << std_acc[0] << " " << std_acc[1] << " " << std_acc[2] << std::endl;
    if (std_acc.maxCoeff() > IMU_Std)
    {
        std::cout << "[ ESKF ] Big acc std: " << std_acc[0] << " " << std_acc[1] << " " << std_acc[2] << std::endl;
        return false;
    }

    state_.time = last_imu_ptr_->timestamp;

    // z-axis
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    // Eigen::Matrix3d R_I_G;
    // R_I_G.block<3, 1>(0, 0) = x_axis;
    // R_I_G.block<3, 1>(0, 1) = y_axis;
    // R_I_G.block<3, 1>(0, 2) = z_axis;

    // state_.R_q = Eigen::Quaterniond(R_I_G);

    stateInit_ = true;
    ROS_ERROR("init state finish");
    return true;
}

bool EkfEstimator::process_IMU_Data(IMUDataPtr imu_data_ptr)
{
    if (!stateInit_)
    {
        imu_buf_.push_back(imu_data_ptr);
        if (imu_buf_.size() > IMU_BUF_SIZE)
            imu_buf_.pop_front();
        return false;
    }
    // std::cout<<"[ ESKF ] imu buffer size: "<<imu_buf_.size()<<std::endl;
    // new imu_data_ptr = std::make_shared<sensor_msgs::Imu>(*imu_msg);
    // 创建一个新的指针，指向imu_msg的内容

    PredictByImu(last_imu_ptr_, imu_data_ptr);
    last_imu_ptr_ = imu_data_ptr;
    return true;
}

void EkfEstimator::PredictByImu(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr)
{

    // mean prediction
    double dt = cur_imu_ptr->timestamp - last_imu_ptr->timestamp;
    double dt_2 = dt * dt;

    // timestamp
    state_.time = cur_imu_ptr->timestamp;

    // p v R
    Eigen::Vector3d acc_unbias = 0.5 * (last_imu_ptr->acc + cur_imu_ptr->acc) - state_.ba;
    Eigen::Vector3d gyr_unbias = 0.5 * (last_imu_ptr->gyro + cur_imu_ptr->gyro) - state_.bg;

    // std::cout << "acc_unbias " << acc_unbias.transpose() << std::endl;
    // std::cout << "gyr_unbias " << gyr_unbias.transpose()  << std::endl;

    Eigen::Vector3d acc_nominal = state_.R_q * state_.R_imu * acc_unbias + gravity_;

    // std::cout << "acc_nominal " << acc_nominal.transpose()  << std::endl;
    // std::cout << "state_.v " << state_.v.transpose()  << std::endl;

    state_.p += state_.v * dt + 0.5 * acc_nominal * dt_2; // 0.5
    state_.v += acc_nominal * dt;
    const Eigen::Vector3d omg = state_.R_imu * (gyr_unbias)*dt;           // 2.0
    Eigen::Quaterniond dq(1.0, omg(0) / 2.0, omg(1) / 2.0, omg(2) / 2.0); // 2
    // Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    const auto &dR = Utility::delta_rot_mat(omg);
    if (omg.norm() > DBL_EPSILON)
    {
        // dR = Eigen::AngleAxisd(omg.norm(), omg.normalized()).toRotationMatrix();
        state_.R_q = state_.R_q * dR;
    }

    
    

    state_.R_imu = state_.R_imu;

    // variance propogation
    Eigen::Matrix3d mI = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd mF = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);

    // dq
    if (omg.norm() > DBL_EPSILON)
    {
        mF.block<3, 3>(StateIndex::R, StateIndex::R) = dR.transpose(); // 3 //dq.toRotationMatrix().transpose() //mI - Utility::SkewSymmetric(imuGyro - state_.bg) * dt
    }
    else
    {
        mF.block<3, 3>(StateIndex::R, StateIndex::R) = mI;
    }
    mF.block<3, 3>(StateIndex::R, StateIndex::BG) = -state_.R_imu * dt;

    if (IF_IMU_MOUNT_ERROR)                                                                                                     // 3
        mF.block<3, 3>(StateIndex::R, StateIndex::IMU_INSTALL_ANGLE) = -state_.R_imu * Utility::SkewSymmetric(gyr_unbias) * dt; // 3
                                                                                                                                // dp

    mF.block<3, 3>(StateIndex::P, StateIndex::P) = mI;      // 3
    mF.block<3, 3>(StateIndex::P, StateIndex::V) = dt * mI; // 3
                                                            // dv
    mF.block<3, 3>(StateIndex::V, StateIndex::R) =
        -dt * state_.R_q * Utility::SkewSymmetric(state_.R_imu * (acc_unbias)); // 3
    mF.block<3, 3>(StateIndex::V, StateIndex::V) = mI;                          // 3
    mF.block<3, 3>(StateIndex::V, StateIndex::BA) = -state_.R_q * state_.R_imu * dt;

    if (IF_IMU_MOUNT_ERROR)                                                                                                                    // 3
        mF.block<3, 3>(StateIndex::V, StateIndex::IMU_INSTALL_ANGLE) = -state_.R_q * state_.R_imu * Utility::SkewSymmetric((acc_unbias)) * dt; // 3
                                                                                                                                               // dba
    mF.block<3, 3>(StateIndex::BA, StateIndex::BA) = mI;                                                                                       // 3
                                                                                                                                               // dbg
    mF.block<3, 3>(StateIndex::BG, StateIndex::BG) = mI;                                                                                       // 3
                                                                                                                                               // d_imu install angle
    mF.block<3, 3>(StateIndex::IMU_INSTALL_ANGLE, StateIndex::IMU_INSTALL_ANGLE) = mI;
    // 3
    mF(StateIndex::WS, StateIndex::WS) = 1.0; // 1

    Eigen::MatrixXd mU = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateNoiseIndex::NOISE_TOTAL);
    // dq
    mU.block<3, 3>(StateIndex::R, StateNoiseIndex::GYRO_NOISE) = -mI * dt;        //-mI * dt;                         // 3
                                                                                  // dv
    mU.block<3, 3>(StateIndex::V, StateNoiseIndex::ACC_NOISE) = -state_.R_q * dt; //-state_.R_q * dt; // 3
                                                                                  // dba
    mU.block<3, 3>(StateIndex::BA, StateNoiseIndex::ACC_RANDOM_WALK) = mI * dt;   // 3
                                                                                  // dbg
    mU.block<3, 3>(StateIndex::BG, StateNoiseIndex::GYRO_RANDOM_WALK) = mI * dt;  // 3

    state_.P = mF * state_.P * mF.transpose() + mU * Q_ * mU.transpose();
}

void EkfEstimator::UpdateByWheel(WheelDataPtr wheel_data_ptr)
{
    // intergate to time_now
    //  Eigen::Vector3d acc = state_.R_q * state_.R_imu.inverse() * (acc_ - state_.ba) + gravity_;
    //  Eigen::Vector3d av = state_.R_imu.inverse() * (gyro_ - state_.bg);

    // double dt = time - state_.time;
    // state_.p = state_.p + state_.v * dt + 0.5 * acc * dt * dt; // 0.5
    // state_.v = state_.v + acc * dt;
    // Eigen::Vector3d omg = av;
    // omg = omg * dt / 2.0; // 2.0
    // Eigen::Quaterniond dq(1.0, omg(0), omg(1), omg(2)); // 2
    // state_.R_q = (state_.R_q * dq).normalized();

    // state_.ba = state_.ba;
    // state_.bg = state_.bg;
    // state_.R_imu = state_.R_imu;
    // state_.ws = state_.ws;

    // Eigen::Vector3d wheelSpeed(wheel_msg->twist.twist.linear.x,
    //                            wheel_msg->twist.twist.linear.y,
    //                            wheel_msg->twist.twist.linear.z);

    Eigen::Vector3d wheelSpeed = wheel_data_ptr->speed;

    // update
    Eigen::VectorXd r(3);                                           // 9
    r = wheelSpeed - state_.ws * (state_.R_q.inverse() * state_.v); // 3
    
    Eigen::MatrixXd mH = Eigen::MatrixXd::Zero(3, StateIndex::STATE_TOTAL);                                   // 9
    mH.block<3, 3>(0, StateIndex::R) = Utility::SkewSymmetric(state_.ws * (state_.R_q.inverse() * state_.v)); // 3

    mH.block<3, 3>(0, StateIndex::V) = state_.ws * state_.R_q.transpose(); // 3

    if (IF_WS_ERROR)
        mH.block<3, 1>(0, StateIndex::WS) = state_.R_q.inverse() * state_.v; // 1

    Eigen::MatrixXd mS = mH * state_.P * mH.transpose() + whl_Rm_;
    Eigen::MatrixXd mK = state_.P * mH.transpose() * mS.inverse();
    Eigen::VectorXd dx = mK * r;

    // state_.R_q = state_.R_q * Eigen::Quaterniond(1.0, dx(StateIndex::R) / 2.0, // 2.0
    //                                          dx(StateIndex::R + 1) / 2.0, dx(StateIndex::R + 2) / 2.0); // 2.0

    const Eigen::Vector3d omg = dx.block<3, 1>(StateIndex::R, 0);         // 2.0
    Eigen::Quaterniond dq(1.0, omg(0) / 2.0, omg(1) / 2.0, omg(2) / 2.0); // 2

    if (dx.block<3, 1>(StateIndex::R, 0).norm() > DBL_EPSILON)
    {
        // dx(StateIndex::R + 2, 0) = 0;
        state_.R_q *= v_expmap(dx.block<3, 1>(StateIndex::R, 0));
        // state_.R_q = state_.R_q * dq;
    }
    // std::cout<<"state_.P"<<state_.P.block<3, 3>(StateIndex::P, StateIndex::P)<<std::endl;
    // state_.R_q.normalize();
    // if(state_.P.block<3, 3>(StateIndex::P, StateIndex::P).norm() < 1)
    // {
    //     state_.p += dx.segment<3>(StateIndex::P); // 3
    // }
    // state_.p += dx.segment<3>(StateIndex::P); // 3
    state_.v += dx.segment<3>(StateIndex::V); // 3
    state_.P = (Eigen::MatrixXd::Identity(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL) - mK * mH) *
               state_.P;
    if (!parameter_lock)
    {
        state_.ba += dx.segment<3>(StateIndex::BA); // 3
        state_.bg += dx.segment<3>(StateIndex::BG); // 3
        // std::cout<<"ba bg: "<<state_.ba.transpose()<<" "<<state_.bg.transpose()<<std::endl;
        if (IF_IMU_MOUNT_ERROR && dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0).norm() > DBL_EPSILON)
        {
            state_.R_imu *= v_expmap(dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0));
            // state_.R_q = state_.R_q * dq;
        }
      
        if (IF_WS_ERROR)
            state_.ws += dx(StateIndex::WS); // 1
    }
}

// UpdateByGps
void EkfEstimator::UpdateByGps(GNSSDataPtr gnss_data_ptr)
{
    ROS_INFO("update by gps");
    bool return_flag = false;
    if (!init_time_)
    {
        init_time_ = gnss_data_ptr->timestamp;
    }
    // std::cout << "init_time " <<init_time_<<std::endl;

    // gnss_count++;
    // if (gnss_count < 10)
    // {
    //     return_flag = true;
    // }
    // else
    // {
    //     gnss_count = 0;
    // }
    Eigen::Vector3d gpsPosition, lla;

    // lla<<gps_msg->pose.position.x, gps_msg->pose.position.y, gps_msg->pose.position.z;
    // gpsPosition = lla;
    // Utility::convert_lla_to_enu(init_lla_, gnss_data_ptr->lla, &gpsPosition);

    Eigen::VectorXd r(3);       // 9
    r = gpsPosition - state_.p; // 3

    Eigen::MatrixXd mH = Eigen::MatrixXd::Zero(3, StateIndex::STATE_TOTAL); // 9
    mH.block<3, 3>(0, StateIndex::P) = Eigen::Matrix3d::Identity();         // 3
    Eigen::Matrix3d V = gnss_data_ptr->cov;

    Eigen::MatrixXd mS = mH * state_.P * mH.transpose() + gps_Rm_;
    Eigen::MatrixXd mK = state_.P * mH.transpose() * (mH * state_.P * mH.transpose() + V).inverse();
    Eigen::VectorXd dx = mK * r;

    // state_.R_q = state_.R_q * Eigen::Quaterniond(1.0, dx(StateIndex::R) / 2.0, // 2.0
    //                                          dx(StateIndex::R + 1) / 2.0, dx(StateIndex::R + 2) / 2.0); // 2.0

    const Eigen::Vector3d omg = dx.block<3, 1>(StateIndex::R, 0);         // 2.0
    Eigen::Quaterniond dq(1.0, omg(0) / 2.0, omg(1) / 2.0, omg(2) / 2.0); // 2

    if (dx.block<3, 1>(StateIndex::R, 0).norm() > DBL_EPSILON)
    {
        state_.R_q *= v_expmap(dx.block<3, 1>(StateIndex::R, 0));
        // state_.R_q = state_.R_q * dq;
    }

    // state_.R_q.normalize();
    // state_.p += dx.segment<3>(StateIndex::P); // 3
    state_.v += dx.segment<3>(StateIndex::V); // 3
    state_.P = (Eigen::MatrixXd::Identity(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL) - mK * mH) *
               state_.P;

    if (!parameter_lock)
    {
        state_.ba += dx.segment<3>(StateIndex::BA); // 3
        state_.bg += dx.segment<3>(StateIndex::BG);
        if (IF_IMU_MOUNT_ERROR && dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0).norm() > DBL_EPSILON)
        {
            state_.R_imu *= v_expmap(dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0));
            // state_.R_q = state_.R_q * dq;
        }
        // if(dx(StateIndex::WS) > 1e-6)
        // {
        //     state_.ws += dx(StateIndex::WS); // 1
        // }
        // 3
        if (IF_WS_ERROR)
            state_.ws += dx(StateIndex::WS); // 1
    }
}

void EkfEstimator::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{

    IMUDataPtr imu_data_ptr = std::make_shared<IMUData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyro[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;
    // ROS_INFO("deal with imu. time stamp: %lf", imu_msg->header.stamp.toSec());

    process_IMU_Data(imu_data_ptr);
    // if (!stateInit_)
    //     return;

    if (!stateInit_)
    {

        if (imu_buf_.size() < IMU_BUF_SIZE)
        {
            std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
            return;
        }

        // ROS_INFO("imu_buf_ size %d", imu_buf_.size());
        last_imu_ptr_ = imu_buf_.back();
        // ROS_INFO("last_imu_ptr_ timestamp %lf", last_imu_ptr_->timestamp);

        if (!InitState())
            return;
        // lla << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude;
        // convert_lla_to_enu(init_lla_,lla, &gpsPosition);

        stateInit_ = true;
    }
    
    Publish();

    // if(!init_gps && IF_USE_GPS) return;
    // m_buf_.lock();
    // imu_buf_.emplace(imu_msg);
    // while (imu_buf_.size() > 1000) {
    //     imu_buf_.pop();
    //     ROS_WARN("throw imu measurement! %lf", imu_msg->header.stamp.toSec());
    // }
    // m_buf_.unlock();

    /*
    if (!stateInit_) {
        InitState(time);
    }
    Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);
    Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z);

    PredictByImu(acc, gyro, time);
    Publish();
    */
}

// void EkfEstimator::WheelCallback(const cyber_msgs::SpeedFeedbackConstPtr &wheel_msg)
void EkfEstimator::WheelCallback(const diankong::VehicleFeedbackConstPtr &wheel_msg)
{
    if (!IF_USE_WHL)
    {
        // std::cout<<"[ ESKF ] Wheel data."<<std::endl;
        return;
    }
    double timestamp = ros::Time(wheel_msg->Hd.time / 1000000000, wheel_msg->Hd.time % 1000000000).toSec();
    WheelDataPtr wheel_data_ptr = std::make_shared<WHLData>();
    // wheel_data_ptr->timestamp = wheel_msg->header.stamp.toSec();
    // wheel_data_ptr->speed[0] = wheel_msg->speed_cms / 100.0;
    // wheel_data_ptr->speed[1] = 0;
    // wheel_data_ptr->speed[2] = 0;

    wheel_data_ptr->timestamp = timestamp;
    wheel_data_ptr->speed[0] =  wheel_msg->speed / 3.6;
    if (wheel_msg->gear == 3)
    {
        wheel_data_ptr->speed[0] = - wheel_msg->speed / 3.6;
    }
    wheel_data_ptr->speed[1] = 0;
    wheel_data_ptr->speed[2] = 0;

    if (!stateInit_)
    {
        std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
        return;
    }

    // if (std::abs(last_imu_ptr_->timestamp - wheel_msg->header.stamp.toSec()) > 0.2)
    // {
    //     std::cout << "[ ESKF ] WHL and IMU are not sychonized." << std::endl;
    //     return;
    // }

    // ROS_INFO("deal with wheel. time stamp: %lf", wheel_msg->header.stamp.toSec());
    UpdateByWheel(wheel_data_ptr);
    // double time = wheel_msg->header.stamp.toSec();
    // ROS_INFO("deal with wheel. time stamp: %lf", wheel_msg->header.stamp.toSec());
    // process_WHL_Data(wheel_msg);

    // is_update = true;
    // odom_buf_.pop();
}

void EkfEstimator::GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg)
{

    bool return_flag = false;
    if (!init_time_)
    {
        init_time_ = gps_msg->header.stamp.toSec();
    }

    GNSSDataPtr gnss_data_ptr = std::make_shared<GNSSData>();
    gnss_data_ptr->timestamp = gps_msg->header.stamp.toSec();
    gnss_data_ptr->lla[0] = gps_msg->latitude;
    gnss_data_ptr->lla[1] = gps_msg->longitude;
    gnss_data_ptr->lla[2] = gps_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg->position_covariance.data());

    if (gps_msg->status.status != 2 )
    {
        parameter_lock = true;
        std::cout << "[ ESKF ] Bad GNSS data." << std::endl;
        return;
    }
    
    // parameter_lock = false;
    // ROS_INFO("step 1 ");
    
    // if (!stateInit_)
    // {

    //     if (imu_buf_.size() < IMU_BUF_SIZE)
    //     {
    //         std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
    //         return;
    //     }

    //     // ROS_INFO("imu_buf_ size %d", imu_buf_.size());
    //     last_imu_ptr_ = imu_buf_.back();
    //     // ROS_INFO("last_imu_ptr_ timestamp %lf", last_imu_ptr_->timestamp);
    //     if (std::abs(last_imu_ptr_->timestamp - gps_msg->header.stamp.toSec()) > 0.1)
    //     {
    //         std::cout << "[ ESKF ] GNSS and IMU are not sychonized." << std::endl;
    //         return;
    //     }
    //     if (!InitState())
    //         return;
    //     // lla << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude;
    //     // convert_lla_to_enu(init_lla_,lla, &gpsPosition);

    //     init_lla_ = gnss_data_ptr->lla;
    //     stateInit_ = true;
    //     ROS_INFO("init lla %f %f %f", init_lla_[0], init_lla_[1], init_lla_[2]);
    // }
    // ROS_INFO("deal with gps. time stamp: %lf", gps_msg->header.stamp.toSec());

    // if (!stateInit_gps)
    // {
    //     init_lla_ = gnss_data_ptr->lla;
    //     stateInit_gps = true;
    //     ROS_INFO("init lla %f %f %f", init_lla_[0], init_lla_[1], init_lla_[2]);
    // }
    Eigen::Vector3d p_G_GNSS, lla;
    // Utility::convert_lla_to_enu(init_lla_, gnss_data_ptr->lla, &p_G_GNSS);

    if (stateInit_)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = gps_msg->header;
        pose_stamped.pose.position.x = p_G_GNSS[0];
        pose_stamped.pose.position.y = p_G_GNSS[1];
        pose_stamped.pose.position.z = p_G_GNSS[2];
        // gnss_path_.header = pose_stamped.header;
        gnss_path_.poses.push_back(pose_stamped);
        gnss_path_pub_.publish(gnss_path_);

        // file_gt_lla << std::fixed << std::setprecision(15)
        //             << gnss_data_ptr->lla[1] << ","
        //             << gnss_data_ptr->lla[0] << ","
        //             << gnss_data_ptr->lla[2] << " ";

        // file_gt_xyz << std::fixed << std::setprecision(15)
        //             << gnss_data_ptr->timestamp << ", "
        //             << p_G_GNSS[0] << ", "
        //             << p_G_GNSS[1] << ", "
        //             << p_G_GNSS[2] << std::endl;
    }
    //deny after 20s
    if (gps_msg->header.stamp.toSec() - init_time_ > 40)
    {
        // modified_msg.status.status = 0;
        std::cout << "[ ESKF ] Deny Time!" << std::endl;
        // last_deny_cnt = deny_cnt;
        return;
    }
    if(!IF_USE_GPS)
    {
        ROS_INFO("no use gps");
        return;
    }
    if (IF_GPS_DENY)
    {
        double time_now = gps_msg->header.stamp.toSec();
        for (auto time : deny_time)
        {
            if (time_now - init_time_ > time(0) && time_now - init_time_ < time(1))
            {
                // modified_msg.status.status = 0;
                // std::cout << "[ ESKF ] Deny Time!" << std::endl;
                // last_deny_cnt = deny_cnt;
                parameter_lock = true;
                return_flag = true;
                break;
            }
        }
    }

    // 如果covariance太大，认为是错误的数据，不使用
    if (gnss_data_ptr->cov(0, 0) > 1 || gnss_data_ptr->cov(1, 1) > 1 )
    {   
        ROS_INFO("cov too big");
        parameter_lock = true;
        return_flag = true;
    }

    if (return_flag || !stateInit_ )
    {
        return;
    }

    // file_gt_ << std::fixed << std::setprecision(15)
    // << gnss_data_ptr->timestamp << ", "
    // << gnss_data_ptr->lla[0] << ", "
    // << gnss_data_ptr->lla[1] << ", "
    // << gnss_data_ptr->lla[2] << std::endl;

    // file_gt_lla << std::fixed << std::setprecision(15)
    //         << p_G_GNSS[0] << ", "
    //             << p_G_GNSS[1] << ", "
    //             << p_G_GNSS[2] << std::endl;

    if (std::abs(last_imu_ptr_->timestamp - gps_msg->header.stamp.toSec()) > 0.1)
    {
        std::cout << "[ ESKF ] GNSS and IMU are not sychonized." << std::endl;
        return;
    }

    UpdateByGps(gnss_data_ptr);
    // return true;

    // if(!init_gps )
    // {
    //     state_.p = Eigen::Vector3d(gps_msg->pose.position.x, gps_msg->pose.position.y, gps_msg->pose.position.z);
    //     init_gps = true;
    //     return;
    // }

    // m_buf_.lock();
    // gps_buf_.emplace(gps_msg);
    // while (gps_buf_.size() > 1000) {
    //     gps_buf_.pop();
    //     ROS_WARN("throw gps measurement! %lf", gps_msg->header.stamp.toSec());
    // }
    // m_buf_.unlock();
}

void EkfEstimator::RosNodeRegistration(ros::NodeHandle &n)
{
    // sub_imu_ = n.subscribe("/Inertial/imu/data", 10, &EkfEstimator::ImuCallback, this,
                        //    ros::TransportHints().tcpNoDelay());
    sub_imu_ = n.subscribe("/wit/imu",10,&EkfEstimator::ImuCallback,this,ros::TransportHints().tcpNoDelay());
    // sub_wheel_ = n.subscribe("/rock_can/speed_feedback", 10, &EkfEstimator::WheelCallback, this,
                            //  ros::TransportHints().tcpNoDelay());
    sub_wheel_ = n.subscribe("/diankong/full_vehicle_feedback",10,&EkfEstimator::WheelCallback,this,ros::TransportHints().tcpNoDelay());
    // sub_gps_ = n.subscribe("/Inertial/gps/fix", 10, &EkfEstimator::GpsCallback, this,
                        //    ros::TransportHints().tcpNoDelay());
    sub_gps_ = n.subscribe("/strong/fix",10,&EkfEstimator::GpsCallback,this,ros::TransportHints().tcpNoDelay());
    pub_path_ = n.advertise<nav_msgs::Path>("/ekf_path", 10);
    pub_odometry_ = n.advertise<nav_msgs::Odometry>("/ekf_odom", 10);
    gnss_path_pub_ = n.advertise<nav_msgs::Path>("/gnss_path", 10);
    path_.header.frame_id = "world";
    gnss_path_.header.frame_id = "world";
}

void EkfEstimator::Publish()
{
    double time = state_.time;
    Eigen::Vector3d position = state_.p;
    Eigen::Vector3d velocity = state_.v;

    // publish the odometry
    std::string fixed_id = "world";
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = fixed_id;
    odom_msg.header.stamp = ros::Time(time);
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = state_.R_q;
    T_wb.translation() = position;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(velocity, odom_msg.twist.twist.linear);
    Eigen::Matrix3d P_pp = state_.P.block<3, 3>(StateIndex::P, StateIndex::P); // position covariance
    Eigen::Matrix3d P_po = state_.P.block<3, 3>(StateIndex::P, StateIndex::R); // position rotation covariance
    Eigen::Matrix3d P_op = state_.P.block<3, 3>(StateIndex::R, StateIndex::P); // rotation position covariance
    Eigen::Matrix3d P_oo = state_.P.block<3, 3>(StateIndex::R, StateIndex::R); // rotation covariance
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
    pub_odometry_.publish(odom_msg);

    // std::cout<<state_.R_q<<std::endl;

    // save state p q lla
    Eigen::Vector3d lla;
    // Utility::convert_enu_to_lla(init_lla_, state_.p, &lla);
    file_our_lla << std::fixed << std::setprecision(15) << lla[1] << "," << lla[0] << "," << lla[2] << " ";

    Eigen::Quaterniond q_G_I(state_.R_q);
    q_G_I.normalize();
    // Eigen::AngleAxisd rotation_vector(rotation_matrix);
    // Eigen::Vector3d eulerAngle=state_.R_q.eulerAngles(2,1,0);
    // eulerAngle  = eulerAngle*180.0/M_PI;
    Eigen::Quaterniond q_G_I_imu(state_.R_imu);
    // Eigen::AngleAxisd imu_rotation_vector(state_.R_imu);
    // Eigen::Vector3d eulerAngle_imu=state_.R_imu.eulerAngles(2,1,0);
    // eulerAngle_imu  = eulerAngle_imu*180.0/M_PI;
    Eigen::VectorXd bias_cov = state_.P.diagonal().tail(10).transpose();
    file_our_state_ << std::fixed << std::setprecision(15) << state_.time << ", "
                    << state_.p[0] << ", " << state_.p[1] << ", " << state_.p[2] << ", "
                    << q_G_I.x() << ", " << q_G_I.y() << ", " << q_G_I.z() << ", " << q_G_I.w() << ", "
                    << state_.v[0] << ", " << state_.v[1] << ", " << state_.v[2] << ", "
                    << state_.ba[0] << ", " << state_.ba[1] << ", " << state_.ba[2] << ", "
                    << state_.bg[0] << ", " << state_.bg[1] << ", " << state_.bg[2] << ", "
                    << bias_cov[0] << ", " << bias_cov[1] << ", " << bias_cov[2] << ", " << bias_cov[3] << ", " << bias_cov[4] << ", " << bias_cov[5] << ", " << bias_cov[6] << ", " << bias_cov[7] << ", " << bias_cov[8] << ", " << bias_cov[9] << ", "
                    << q_G_I_imu.x() << ", " << q_G_I_imu.y() << ", " << q_G_I_imu.z() << ", " << q_G_I_imu.w() << ", "
                    << state_.ws
                    << std::endl;

    // std::cout << "bias of acc: " << state_.ba.transpose() << std::endl;
    // std::cout << "bias of gyr: " << state_.bg.transpose() << std::endl;
    // std::cout << "ws " << state_.ws << std::endl;
    // std::cout << "R_imu " << state_.R_imu << std::endl;

    // pub_odometry_.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(time);
    pose_stamped.pose = odom_msg.pose.pose;
    path_.poses.push_back(pose_stamped);
    pub_path_.publish(path_);
}