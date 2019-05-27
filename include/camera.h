#ifndef FLIGHT_SIM_CAMERA_H_
#define FLIGHT_SIM_CAMERA_H_
#include <memory>
#include<random>
#include<ctime>
#include <opencv2/core/core.hpp>
#include "cIMUOdo.h"

namespace flight_sim{

class Ground;

class Camera{

public:
    // struct IMUPose{
    //     double initial_pt_[3];
    //     double initial_angle_[3];
    //     double initial_v_[3];
    // };

    struct IMUParam{
        double a_mean_[3];
        double a_std_[3];
        double w_mean_[3];
        double w_std_[3];
        std::string acc_error_model_file_;
        std::string gro_error_model_file_;
    };

    void Init(const IMUParam &params, const cv::Mat &K, const std::shared_ptr<Ground> &ground );
    void SetInitPose(const double init_pose[9]);                // init_pose = {angle, position, victory}
    void UpdatePose();
    bool GetCurrentImage(cv::Mat &result_img, cv::Mat &result_road_img);
    bool GetCurrentImage(cv::Mat &result_img);

    // void ShowRoadOnGround(const Ground &ground, const cv::Mat &road_img, cv::Mat &draw_img);

    inline const cv::Mat &get_H_c_2_g() const {return H_inv_;} 

private:
    void InitIMU(const IMUParam &params);
    void GenerateIMUData();
    cv::Point2d CameraCorToGeoCor(const cv::Point2d &pixel_pt);    // 将当前相机的图像点转换到地理坐标系下
    
    // state 
    cv::Mat R_;
    cv::Mat t_;
    cv::Mat K_;
    cv::Mat H_;
    cv::Mat H_inv_;

    // random
    //产生随机数引擎，采用time作为种子，以确保每次运行程序都会得到不同的结果
    std::default_random_engine random_engine_;
    std::normal_distribution<double> n_distribution_a_[3];
    std::normal_distribution<double> n_distribution_w_[3];

    // imu params
    IMUParam imu_params_;

    // imu
    cIMUOdo imuOdo_;
    const double STD_GRAVITY = 0;
    const double dt_ = 0.03333;
    double acc_[3];
    double gro_[3];
    double time_;

    // ground
    std::shared_ptr<Ground> ground_;
};
    
} // flight_sim

#endif