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


int main(int argc, char *argv[]){
    if(argc<3){
        std::cout<<"Usage: test_ground [sat_file_name] [road_file_name]"<<std::endl;
        exit(-1);
    }
    std::shared_ptr<flight_sim::Ground> ground (new flight_sim::Ground );
    if(!ground->Init(argv[1],argv[2], 4.0)){
        std::cout<<"Fail to load ground file"<<std::endl;
        return -1;
    }
    ground->Print();

    // 301500, 4664786, 366349-301500, 4664786-4652999
    // cv::Rect_<double> active_area(306500, 4669786, 5000, 15000);
    // cv::Rect_<double> active_area(301500+500, 4664786-500, 366349-301500-1000, 4664786-4652999-1000);
    cv::Rect_<double> active_area_sat(301500+1000, 4664786-1000, 366349-301500-2000, 4664786-4652999-2000);
    cv::Rect_<double> active_area_road(301500+1000, 4664786-1000 - 213, 366349-301500-2000, 4664786-4652999-2000);
    bool sat_flag = ground->UpdateActiveSatImage(active_area_sat);
    bool road_flag = ground->UpdateActiveRoadImage(active_area_road,1);

    std::cout<<"sat: "<<sat_flag<<std::endl;
    std::cout<<"road: "<<road_flag<<std::endl;
    

    ground->ShowActiveArea(0.5);

    return 0;
}