#include "mrsid/sid.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(){
    flight_sim::MrSid li_sid;
    if(!li_sid.Init("9.sid")){
        std::cout<<"Can't open file"<<std::endl;
        return -1;
    }
    li_sid.Print();

    cv::Mat result_img;
    int x = 321000; 
    int y = 4.6815e+06;
    int width = 363000 - x;
    int height = y - 4.665e+06;
    int count = 0;

    // 将一个大的区域分散为多个小区域
    cv::namedWindow("src", CV_WINDOW_NORMAL);
    for(int i=0; i<width; i+=10000){
        for(int j=0; j<height; j+=5000){
            if(li_sid.GetROISatImage(x + i /* - 2 */, y - j, 10000, 6000, 1.2, result_img)){
                // char tmp[40];
                // sprintf(tmp, "ref_img_%d_%d.jpg", i, j);
                // std::cout<<"Save image: "<< std::string(tmp)<<std::endl;
                // cv::imwrite(tmp, result_img);
                
                cv::imshow("src", result_img);
                cv::waitKey(0);
            }
        }
    }
    
    // std::cout<<"image_width = "<<result_img.cols<<std::endl;

    

    // double t = cv::getTickCount();
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