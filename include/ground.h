#ifndef FLIGHT_SIM_GROUND_H_
#define FLIGHT_SIM_GROUND_H_

#include <opencv2/core/core.hpp>
#include "shp/shp.h"
#include "mrsid/sid.h"

namespace flight_sim{

class Ground{
public:
    // 设置卫星图文件.sid和道路图文件.shp，并提取文件边界/坐标系信息，记录在对应的成员变量中
    bool Init(const std::string &sid_file, 
                const std::string &shp_file, 
                const double resolution);  

    int UpdateActiveSatImage(const cv::Rect_<double> &bound);                                    // 根据相机的状态（姿态/位置/速度），提取当前相机一段时间内可能观测到的卫星图区域
    int UpdateActiveRoadImage(const cv::Rect_<double> &bound, const int road_width = 7);         // 根据相机的状态（姿态/位置/速度），提取当前相机一段时间内可能观测到的道路图区域
    // 同时运行UpdateActiveSatImage和UpdateActiveRoadImage
    int UpdateActiveSatAndRoadImage(const cv::Rect_<double> &sat_bound, 
                                    const cv::Rect_<double> &road_bound,
                                    const int road_width = 7);      

    // 返回当前提取的卫星图区域active_sat_
    inline const cv::Mat & GetActiveSatImage() const {return active_sat_img_;}                                                
    // 返回当前提取的卫星图区域在地理坐标系中的up-left的坐标
    inline const cv::Vec4d & GetActiveSatGeoBound() const {return sat_geo_bound_;}                                         
    // 返回当前提取的道路图区域active_sat_
    inline const cv::Mat & GetActiveRoadImage() const {return active_road_img_;}   
    // 返回当前提取的道路图区域在地理坐标系中的up-left的坐标                                            
    inline const cv::Vec4d & GetActiveRoadGeoBound() const {return road_geo_bound_;}   

    // 给定中心点坐标，移动方向，图像尺寸，计算active区域的边界
    cv::Rect_<double> CalculateActiveArea(const cv::Point2d &center_pt, 
                                            const double width, 
                                            const double height, 
                                            const cv::Point2d &v = cv::Point2d(0,0));

    // 地理坐标系转换为当前活跃区域的图像坐标系
    cv::Point2d SatGeoCorToActiveImageCor(const cv::Point2d &geo_pt);      
    cv::Point2d RoadGeoCorToActiveImageCor(const cv::Point2d &geo_pt);     

    void Print();      
    void ShowActiveArea(double scale =0.1, int time = 0);                                       

private:
    cv::Mat active_sat_img_;
    cv::Mat active_road_img_;
    
    cv::Vec4d road_geo_bound_;                  // 当前active区域的边界（地理坐标系，minx,miny,maxx,maxy）
    cv::Vec4d sat_geo_bound_;

    cv::Vec4d geo_bound_;                       // sid和shp的地理边界较小的一个(minx,miny,maxx,maxy)

    SHP shp_;
    MrSid sid_; 

    double resolution_;
};
    
} // flight_sim



#endif