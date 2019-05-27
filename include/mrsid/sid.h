#ifndef FLIGHT_SIM_SID_H_
#define FLIGHT_SIM_SID_H_
#include <opencv2/core/core.hpp>


namespace flight_sim {

class MrSid{
public:    
    bool Init(const std::string & file_name);
    bool GetROISatImage(const double x, const double y,                 // 地理坐标系
                     const double width, const double height, 
                     const double resolution, cv::Mat &result_img);     // 这里给定的resolution如果不是最高分辨率的2的倍数，会被转换到最近的2的倍数

    inline double get_resolution() const {
        if(res_[0]>0)
            return res_[0];
        else
            return -res_[0];
    }
    inline cv::Vec4d get_geo_bound() const {return cv::Vec4d(bound_[0], bound_[1], bound_[2], bound_[3]);}
    // debug
    void Print();
    
private:
    int width_;
    int height_;
    int channels_; 
    int data_type_;

    double min_magnification_;
    double max_magnification_;

    std::string coordination_;
    double bound_[4];                   // left-up-x, left-up-y, width, height
    double res_[2];                     // 分辨率x,y

    std::string file_name_;
};
    
} // namespace flight_sim




#endif