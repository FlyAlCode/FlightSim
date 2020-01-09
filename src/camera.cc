#include "camera.h"
#include "ground.h"
#include <opencv2/imgproc/imgproc.hpp>


// debug
#include <iostream>


namespace flight_sim{
void Camera::Init(const IMUParam &params, const cv::Mat &K, const std::shared_ptr<Ground> &ground ){
    InitIMU(params);
    K.copyTo(K_);
    ground_ = ground;
}


void Camera::SetInitPose(const double pose[9]){
    // set initial state
    double t_data[3] = {pose[3], pose[4], pose[5]};
    cv::Mat t_tmp(3, 1, CV_64F, t_data);
    t_tmp.copyTo(t_);
    
    // initilize R_
    Eigen::Vector3d initial_angle_tmp;
    initial_angle_tmp << pose[2],         // Z,Y,X
                         pose[1],
                         pose[0];
    Eigen::Matrix3d R_tmp;
    R_tmp = ::Eigen::AngleAxisd(initial_angle_tmp[0], ::Eigen::Vector3d::UnitZ())
        * ::Eigen::AngleAxisd(initial_angle_tmp[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(initial_angle_tmp[2], ::Eigen::Vector3d::UnitX());
    cv::Mat Rtmp(3,3,CV_64F);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            Rtmp.at<double>(i,j) = R_tmp(i,j);
        }
    }
    Rtmp.copyTo(R_);
   
    // change coordiantion
    R_ = R_.t();
    t_ = -R_ * t_;

    R_.copyTo(Rtmp);
    // std::cout<<Rtmp<<std::endl;
    t_.copyTo(Rtmp.col(2));
    // std::cout<<Rtmp<<std::endl;
    H_ = K_ * Rtmp;
    H_inv_ = H_.inv();

    Eigen::Matrix< double, 10 , 1> initState;
    Eigen::Quaterniond q; 
    q = R_tmp; 
    initState<<q.w(), q.x(), q.y(), q.z(), pose[6], pose[7], pose[8], pose[3], pose[4], pose[5];
    imuOdo_.setInitState(initState);
}

void Camera::UpdatePose(){
    GenerateIMUData();

    time_ += dt_;
    Eigen::Matrix< double, 3 , 1>  gro;
    gro << gro_[0], gro_[1], gro_[2];
    Eigen::Matrix< double, 3 , 1> acc;
    acc << acc_[0], acc_[1], acc_[2];
    imuOdo_.updateOdo(time_, gro, acc);
        
    Eigen::Matrix< double, 10 , 1> pose;
    imuOdo_.getPose(pose);

    // std::cout<<"current camera pose:\n "<<pose<<std::endl;
   

    // update state of camera(R,t,H)
    Eigen::Quaterniond q(pose(0), pose(1), pose(2), pose(3));
    Eigen::Matrix3d R_tmp = q.toRotationMatrix();
    cv::Mat R_opencv(3,3,CV_64F);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            R_opencv.at<double>(i,j) = R_tmp(i,j);
        }
    }
    // R_opencv = R_opencv.inv();
    R_opencv.copyTo(R_);

    for(int i=0; i<3; i++)
        t_.at<double>(i,0) = pose(7+i);
    
    // change coordiantion
    R_ = R_.t();
    t_ = -R_ * t_;
    cv::Mat R_H_tmp;
    R_.copyTo(R_H_tmp);
    t_.copyTo(R_H_tmp.col(2));
    H_ = K_ * R_H_tmp;    
    H_inv_ = H_.inv();
    H_inv_ = H_inv_/H_inv_.at<double>(2,2);
}

bool Camera::GetCurrentImage(cv::Mat &result_img, cv::Mat &result_road_img){
    // 1 获取当前相机平面的四个角点,并判断其是否超出了当前活跃区域的边界
    bool out_of_bound = false;
    cv::Vec4d geo_bound = ground_->GetActiveSatGeoBound();
    int img_width = K_.at<double>(0,2)*2;
    int img_height = K_.at<double>(1,2)*2;
    cv::Point2d left_up(0, 0);
    cv::Point2d geo_left_up = CameraCorToGeoCor(left_up);
    if(!(geo_left_up.x>=geo_bound[0] && geo_left_up.x<geo_bound[2] && geo_left_up.y>=geo_bound[1] && geo_left_up.y<geo_bound[3]))
        out_of_bound = true;
    
    cv::Point2d right_up(img_width, 0);
    cv::Point2d geo_right_up = CameraCorToGeoCor(right_up);
    if(!(geo_right_up.x>=geo_bound[0] && geo_right_up.x<geo_bound[2] && geo_right_up.y>=geo_bound[1] && geo_right_up.y<geo_bound[3]))
        out_of_bound = true;
    
    cv::Point2d left_down(0, img_height);
    cv::Point2d geo_left_down = CameraCorToGeoCor(left_down);
    if(!(geo_left_down.x>=geo_bound[0] && geo_left_down.x<geo_bound[2] && geo_left_down.y>=geo_bound[1] && geo_left_down.y<geo_bound[3]))
        out_of_bound = true;
    
    cv::Point2d right_down(img_width, img_height);
    cv::Point2d geo_right_down = CameraCorToGeoCor(right_down);
    if(!(geo_right_down.x>=geo_bound[0] && geo_right_down.x<geo_bound[2] && geo_right_down.y>=geo_bound[1] && geo_right_down.y<geo_bound[3]))
        out_of_bound = true;
    
    // 2 如果超出当前活跃区域的边界，重新生成活跃区域
    if(out_of_bound){
        cv::Point2d center_pt = (geo_left_down + geo_left_up + geo_right_down + geo_right_up) / 4.0;
        cv::Rect_<double> active_area = ground_->CalculateActiveArea(center_pt, 10000,5000);
        cv::Rect2d road_active_area(active_area.x, active_area.y-213, active_area.width, active_area.height);
        // test whether the whole coordinate are aligned
        // cv::Rect_<double> active_area = cv::Rect2d(301500, 4.665e+06, 66000, 12000);
        ground_->UpdateActiveSatImage(active_area);
        ground_->UpdateActiveRoadImage(road_active_area);

        // debug
        // ground_->ShowActiveArea(1);
    }
    // 3 获取图像
    if(ground_->GetActiveSatImage().empty())
        return false;
    
    cv::Mat tmp = cv::Mat::zeros(img_height, img_width, ground_->GetActiveSatImage().type());
    cv::Mat tmp_road = cv::Mat::zeros(img_height, img_width, ground_->GetActiveRoadImage().type());
    for(int i=0; i<img_width - 1; i++){
        for(int j=0; j<img_height - 1; j++){
            cv::Point2d img_pt(i, j);
            cv::Point2d dst_pt = ground_->SatGeoCorToActiveImageCor(CameraCorToGeoCor(img_pt));
            if(dst_pt.x<0||dst_pt.y<0||dst_pt.x>ground_->GetActiveSatImage().cols|| dst_pt.y>ground_->GetActiveSatImage().rows)
                return false;
            if(tmp.channels() ==1){                   // here is unsafe
                tmp.at<uchar>(img_pt) = ground_->GetActiveSatImage().at<uchar>(dst_pt);
            }
            if(tmp.channels() == 3){
                tmp.at<cv::Vec3b>(img_pt) = ground_->GetActiveSatImage().at<cv::Vec3b>(dst_pt);
            }

            // road
            if(dst_pt.x<0||dst_pt.y<0||dst_pt.x>ground_->GetActiveRoadImage().cols|| dst_pt.y>ground_->GetActiveRoadImage().rows)
                return false;
            tmp_road.at<uchar>(img_pt) = ground_->GetActiveRoadImage().at<uchar>(dst_pt);
        }
    }
    tmp.copyTo(result_img);
    tmp_road.copyTo(result_road_img);

    return true;
}


bool Camera::GetCurrentImage(cv::Mat &result_img){
    // 1 获取当前相机平面的四个角点,并判断其是否超出了当前活跃区域的边界
    bool out_of_bound = false;
    cv::Vec4b geo_bound = ground_->GetActiveSatGeoBound();
    int img_width = K_.at<double>(0,2)*2;
    int img_height = K_.at<double>(1,2)*2;
    cv::Point2d left_up(0, 0);
    cv::Point2d geo_left_up = CameraCorToGeoCor(left_up);
    if(!(geo_left_up.x>=geo_bound[0] && geo_left_up.x<geo_bound[2] && geo_left_up.y>=geo_bound[1] && geo_left_up.y<geo_bound[3]))
        out_of_bound = true;
    
    cv::Point2d right_up(img_width, 0);
    cv::Point2d geo_right_up = CameraCorToGeoCor(right_up);
    if(!(geo_right_up.x>=geo_bound[0] && geo_right_up.x<geo_bound[2] && geo_right_up.y>=geo_bound[1] && geo_right_up.y<geo_bound[3]))
        out_of_bound = true;
    
    cv::Point2d left_down(0, img_height);
    cv::Point2d geo_left_down = CameraCorToGeoCor(left_down);
    if(!(geo_left_down.x>=geo_bound[0] && geo_left_down.x<geo_bound[2] && geo_left_down.y>=geo_bound[1] && geo_left_down.y<geo_bound[3]))
        out_of_bound = true;
    
    cv::Point2d right_down(img_width, img_height);
    cv::Point2d geo_right_down = CameraCorToGeoCor(right_down);
    if(!(geo_right_down.x>=geo_bound[0] && geo_right_down.x<geo_bound[2] && geo_right_down.y>=geo_bound[1] && geo_right_down.y<geo_bound[3]))
        out_of_bound = true;
    
    // 2 如果超出当前活跃区域的边界，重新生成活跃区域
    if(out_of_bound){
        cv::Point2d center_pt = (geo_left_down + geo_left_up + geo_right_down + geo_right_up) / 4.0;
        cv::Rect_<double> active_area = ground_->CalculateActiveArea(center_pt, 10000,5000);
        ground_->UpdateActiveSatImage(active_area);
    }
    // 3 获取图像
    if(ground_->GetActiveSatImage().empty())
        return false;
    
    cv::Mat tmp = cv::Mat::zeros(img_height, img_width, ground_->GetActiveSatImage().type());
    for(int i=0; i<img_width - 1; i++){
        for(int j=0; j<img_height - 1; j++){
            cv::Point2d img_pt(i, j);
            cv::Point2d dst_pt = ground_->SatGeoCorToActiveImageCor(CameraCorToGeoCor(img_pt));
            if(dst_pt.x<0||dst_pt.y<0||dst_pt.x>ground_->GetActiveSatImage().cols|| dst_pt.y>ground_->GetActiveSatImage().rows)
                return false;
            if(tmp.channels() ==1){                   // here is unsafe
                tmp.at<uchar>(img_pt) = ground_->GetActiveSatImage().at<uchar>(dst_pt);
            }
            if(tmp.channels() == 3){
                tmp.at<cv::Vec3b>(img_pt) = ground_->GetActiveSatImage().at<cv::Vec3b>(dst_pt);
            }
        }
    }
    tmp.copyTo(result_img);

    return true;
}

/**************** private ****************/
void Camera::InitIMU(const IMUParam &params){
    imu_params_ = params;
    Eigen::Matrix< double, 10 , 1> initState;
    initState<< 1,0,0,0,0,0,0,0,0,0;

    Eigen::Matrix< double, 3 , 1> initGravity;
    initGravity<<0,0,STD_GRAVITY;

    time_ = 0;
    bool flag = imuOdo_.init(params.acc_error_model_file_, 
                            params.gro_error_model_file_, 
                            initGravity, 
                            initState, 
                            time_);

    // debug
    // Eigen::Matrix<double, 10, 1> pose;
    // imuOdo.getPose(pose);
    // std::cout<<"initial imu pose: "<<pose<<std::endl;

    // initilize random generator
    random_engine_ = std::default_random_engine(time(0));
    for(int i=0; i<3; i++){
        n_distribution_a_[i] = 
            std::normal_distribution<double>(imu_params_.a_mean_[i], imu_params_.a_std_[i]);
        n_distribution_w_[i] = 
            std::normal_distribution<double>(imu_params_.w_mean_[i], imu_params_.w_std_[i]);
    }

}
void Camera::GenerateIMUData(){
    for(int i=0; i<3; i++){
        acc_[i] =  n_distribution_a_[i](random_engine_);
        gro_[i] = n_distribution_w_[i](random_engine_);
    }
}   

// 将当前相机的图像点转换到地理坐标系下
cv::Point2d Camera::CameraCorToGeoCor(const cv::Point2d &pixel_pt){
    CV_Assert(!H_inv_.empty());
    cv::Mat pixel_pt_mat(3,1,CV_64F);
    pixel_pt_mat.at<double>(0,0) = pixel_pt.x;
    pixel_pt_mat.at<double>(1,0) = pixel_pt.y;
    pixel_pt_mat.at<double>(2,0) = 1;

    cv::Mat geo_h = H_inv_ * pixel_pt_mat;
    return cv::Point2d(geo_h.at<double>(0,0)/geo_h.at<double>(2,0),
                        geo_h.at<double>(1,0)/geo_h.at<double>(2,0));
}   

    
} // flight_sim