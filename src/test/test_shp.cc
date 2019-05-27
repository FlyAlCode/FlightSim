#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "shp/shp.h"

int main(){
    flight_sim::SHP li_shp;
    if(!li_shp.Init("/home/li/DataSet/2013Mosaic/shp/sub_area_shp/epsg2033-10.shp")){
        std::cout<<"Fail to open shp file, please check!!!"<<std::endl;
        exit(-1);
    }
    li_shp.Print();

    cv::Mat road_img;
    double x = 301500;
    double y = 4.665e+06;
    li_shp.GetROIRoadImage(x, y, 10000, 10000, 10.0, 1, road_img);
    cv::imshow("road", road_img);
    cv::waitKey(0);

    // while (li_shp.GetROIRoadImage(x, y, 3000, 3000, 1.0, 7, road_img))  {
    //     cv::imshow("road", road_img);
    //     cv::waitKey(1);
    //     x+=2000;
    //     y+=2000;
    //     std::cout<<"x = "<<x<<" ,y = "<<y<<std::endl;
    // }
    
    return 0;
}
