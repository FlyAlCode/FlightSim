#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "camera.h"
#include "ground.h"

double const PI = 3.1415926;

int main(int argc, char *argv[]){
    if(argc<5){
        std::cout<<"Usage: sat_only [acc_error_model_file] [gro_error_model_file][sat_image_name] [save_path]"<<std::endl;
        exit(-1);
    }
    // set flight params
    flight_sim::Camera::FlightParam param;
    param.acc_error_model_file_ = argv[1];
    param.gro_error_model_file_ = argv[2];

    param.initial_pt_[0]= -750;
    param.initial_pt_[1]= -750;
    param.initial_pt_[2]= -500;
    param.initial_angle_[0]= 0;
    param.initial_angle_[1]= 10.0/180*PI;
    param.initial_angle_[2]= 45.0/180*PI;
    param.initial_v_[0]=30;
    param.initial_v_[1]=30;
    param.initial_v_[2]=0;

    for(int i=0; i<3; i++){
        param.a_mean_[i] = 0;
        param.a_std_[i] = 0.01;
        param.w_mean_[i] = 0;
        param.w_std_[i] = 0.03;
    }

    // create a ground
    flight_sim::Ground ground(1);
    cv::Mat sat_img = cv::imread(argv[3]);
    ground.Init(sat_img);
    // create a camera
    flight_sim::Camera camera;
    double K_data[9] = {450, 0, 250, 0, 450, 250, 0, 0, 1};
    cv::Mat K(3, 3, CV_64F, K_data);
    camera.Init(param, K);

    // save leftup image as a reference
    std::string root_dir(argv[4]);
    std::string leftup_img_name = root_dir + "sat/00.png";
    cv::imwrite(leftup_img_name, sat_img(cv::Rect(0,0,500,500)));


    bool is_started = false;
    bool is_finished = false;    
    std::string sat_img_name;
    int n=10;
    cv::Mat current_img;
    cv::Mat draw_img;

    // cv::VideoWriter v_writer("road_on_ground.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(1500, 1500));

    std::string pose_file_name = root_dir + "pose.txt";
    std::ofstream fout(pose_file_name);
    while(!(is_started && is_finished) && cv::waitKey(0) != 'q'){
        camera.UpdateState();
        bool success = camera.GetCurrentImage(ground, current_img);
        if(is_started==false)
            is_started = success;
        else 
            is_finished = !success;

        if(success){
            // cv::imshow("sat", current_img);
            // camera.ShowRoadOnGround(ground, current_road_img, draw_img);
            // cv::imshow("road on ground",draw_img);
            // cv::waitKey(1);
            // v_writer<<draw_img;

            sat_img_name = root_dir + "sat/" + std::to_string(n) + ".png";
            cv::imwrite(sat_img_name, current_img);
            cv::imshow("current sat image", current_img);

            for(int i=0; i<9; i++){
                fout<<camera.get_H_c_2_g().at<double>(i/3, i%3)<<"  ";
            }
            fout<<std::endl;
            
            ++n;
        }

    }
    // v_writer.release();
    fout.close();
    // std::cout<<"Fineshed......"<<std::endl;
    return 0;
}