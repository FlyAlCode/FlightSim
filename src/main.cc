#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "camera.h"
#include "ground.h"

#include <stdio.h>
#include <stdlib.h>


double const PI = 3.1415926;

void DrawMask(const cv::Mat &src, const cv::Mat &mask, cv::Mat &result){
    CV_Assert(mask.rows==src.rows && mask.cols == mask.rows);
    if(mask.channels()!=1)
        cv::cvtColor(mask, mask, CV_RGB2GRAY);

    src.copyTo(result);
    
    for(int i=0; i<mask.rows; i++){
        for(int j=0; j<mask.cols; j++){
            if(mask.at<uchar>(i,j)!=0)
                result.at<cv::Vec3b>(i,j) = cv::Vec3b(0,255,0);
        }
    }
}

int main(int argc, char *argv[]){
    if(argc<22){
        std::cout<<"Usage: flight_sim [acc_error_model_file] [gro_error_model_file][sat_file_name] [road_file_name] \
                \n      [save_path] [angle_x angle_y angle_z] [position_x position_y position_z] [v_x v_y v_z] \
                 [a_mean_x a_mean_y a_mean_z]  [w_mean_x w_mean_y w_mean_z] [max_image_num]"<<std::endl;
        exit(-1);
    }
    std::shared_ptr<flight_sim::Ground> ground (new flight_sim::Ground );
    if(!ground->Init(argv[3],argv[4], 1.2)){
        std::cout<<"Fail to load ground file"<<std::endl;
        return -1;
    }
    ground->Print();

    // init camera
    flight_sim::Camera::IMUParam imu_params;
    imu_params.acc_error_model_file_ = std::string(argv[1]);
    imu_params.gro_error_model_file_ = std::string(argv[2]);
    for(int i=0; i<3; i++){
        // imu_params.a_mean_[i] = 0;
        imu_params.a_std_[i] = 0.5;
        // imu_params.w_mean_[i] = 0;
        imu_params.w_std_[i] = 0.0;
    }
    imu_params.a_mean_[0] = atof(argv[15]);
    imu_params.a_mean_[1] = atof(argv[16]);
    imu_params.a_mean_[2] = atof(argv[17]);
    imu_params.w_mean_[0] = atof(argv[18]);
    imu_params.w_mean_[1] = atof(argv[19]);
    imu_params.w_mean_[2] = atof(argv[20]);

    for(int i=0; i<3; ++i){
        std::cout<<imu_params.a_mean_[i]<<" ";
    }
    std::cout<<std::endl;
    for(int i=0; i<3; ++i){
        std::cout<<imu_params.w_mean_[i]<<" ";
    }
    std::cout<<std::endl;

    
    flight_sim::Camera camera;

    double K_data[9] = {450, 0, 400, 0, 450, 400, 0, 0, 1};
    cv::Mat K(3, 3, CV_64F, K_data);

    camera.Init(imu_params,K, ground);

    // set initial pose of the camera
    double init_pose[9]/*  = {0,20/180*PI,0,355000,4659000,1000,-200,300,0} */;    // angle, position, v
    // get initial pose from params
    {
        init_pose[0] = atof(argv[6])/* /180*PI */;
        init_pose[1] = atof(argv[7])/* /180*PI */;
        init_pose[2] = atof(argv[8])/* /180*PI */;
        init_pose[3] = atof(argv[9]);
        init_pose[4] = atof(argv[10]);
        init_pose[5] = atof(argv[11]);
        init_pose[6] = atof(argv[12]);
        init_pose[7] = atof(argv[13]);
        init_pose[8] = atof(argv[14]);
    }
    // double init_pose[9];
    // std::cout<<"Please input initial_angle_x:"<<std::endl;
    // std::cin>>init_pose[0];
    // std::cout<<"Please input initial_angle_y:"<<std::endl;
    // std::cin>>init_pose[1];
    // std::cout<<"Please input initial_angle_z:"<<std::endl;
    // std::cin>>init_pose[2];
    // std::cout<<"Please input initial_position_x:"<<std::endl;
    // std::cin>>init_pose[3];
    // std::cout<<"Please input initial_position_y:"<<std::endl;
    // std::cin>>init_pose[4];
    // std::cout<<"Please input initial_position_z:"<<std::endl;
    // std::cin>>init_pose[5];
    // std::cout<<"Please input initial_v_x:"<<std::endl;
    // std::cin>>init_pose[6];
    // std::cout<<"Please input initial_v_y:"<<std::endl;
    // std::cin>>init_pose[7];
    // std::cout<<"Please input initial_v_z:"<<std::endl;
    // std::cin>>init_pose[8];

    camera.SetInitPose(init_pose);

    std::cout<<"************ Start fly!!! *************"<<std::endl;
    cv::Mat current_sat_img, current_road_img;
    // cv::namedWindow("sat");
    // cv::namedWindow("road");
    int img_count = 0;
    std::ofstream fout(std::string(argv[5]) + "H_c2g.txt");
    std::ofstream fout_R(std::string(argv[5]) + "R.txt");
    std::ofstream fout_T(std::string(argv[5]) + "t.txt");
    fout.precision(7);
    fout_R.precision(7);
    fout_T.precision(7);
    int max_img_num = atoi(argv[21]);
    while(cv::waitKey(1)!='q' && img_count<max_img_num){
        camera.UpdatePose();
        if(!camera.GetCurrentImage(current_sat_img, current_road_img))
            break;
        
        // cv::imshow("sat", current_sat_img);
        // cv::imshow("road", current_road_img);
        cv::Mat add_img;
        DrawMask(current_sat_img, current_road_img, add_img);
        cv::imshow("add img", add_img);
        std::cout<<"img number: "<<img_count<<std::endl;

        // write image
        char tmp[20];
        sprintf(tmp, "%.3d", img_count);
        std::string sat_save_name = std::string(argv[5]) + "sat/" + tmp + ".jpg";
        std::string road_save_name = std::string(argv[5]) + "road/" + tmp + ".png";
        // std::cout<<road_save_name<<std::endl;
        cv::imwrite(sat_save_name, current_sat_img);
        cv::imwrite(road_save_name, current_road_img);

        // print pose
        cv::Mat H_c2g = camera.get_H_c_2_g();
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                fout<<H_c2g.at<double>(i,j)<<" ";
            }
        }
        fout<<std::endl;

        cv::Mat R_tmp = camera.get_R();
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                fout_R<<R_tmp.at<double>(i,j)<<" ";
            }
        }
        fout_R<<std::endl;

        cv::Mat t_tmp = camera.get_t();
        // t_tmp 为相机中心在世界坐标系下的坐标
        t_tmp = -R_tmp.t() * t_tmp;
        // std::cout<<t_tmp.t()<<std::endl;
        for(int i=0; i<3; i++){
            fout_T<<t_tmp.at<double>(i,0)<<" ";
        }
        fout_T<<std::endl;

        ++img_count;
    }
    fout.close();
    fout_R.close();
    fout_T.close();


    return 0;
}
