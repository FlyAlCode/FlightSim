#include "mrsid/sid.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(){
    flight_sim::MrSid li_sid;
    if(!li_sid.Init("/home/li/DataSet/2013Mosaic/coq2013_10.sid")){
        std::cout<<"Can't open file"<<std::endl;
        return -1;
    }
    li_sid.Print();

    cv::Mat result_img;
    int x = 301500; 
    int y = 4.665e+06;
    int width = 2000/* 66000 */;
    int height = 2000/* 12000 */;
    int count = 0;
    double t = cv::getTickCount();
    li_sid.GetROISatImage(x,y,width, height, 0.5, result_img);
    cv::imshow("test sid", result_img);
    cv::waitKey(0);


    // while(li_sid.GetROISatImage(x,y,2000,2000, 1.0, result_img)){
    //     // std::cout<<"Fail to get roi image"<<std::endl;
    //     // return -1;
    //     x += 1000;
    //     y += 1000;
    //     cv::imshow("test sid", result_img);
    //     cv::waitKey(0);
    //     ++count;
    // }
    // t = (cv::getTickCount() - t)/cv::getTickFrequency();
    // std::cout<<"average time = "<<t/count<<std::endl;
    
    return 0;
}