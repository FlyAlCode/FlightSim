#include <math.h>
#include "ground.h"
// debug
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

namespace flight_sim{
// 设置卫星图文件.sid和道路图文件.shp，并提取文件边界/坐标系信息，记录在对应的成员变量中
bool Ground::Init(const std::string &sid_file, const std::string &shp_file, const double resolution){
    bool shp_init_success = shp_.Init(shp_file);
    bool sid_init_success = sid_.Init(sid_file);
    if(!(shp_init_success && sid_init_success))
        return false;
    road_geo_bound_ = cv::Vec4d(0,0,0,0);
    sat_geo_bound_ = cv::Vec4d(0,0,0,0);
    
    // 计算最近的2的倍数的分辨率
    double scale = sid_.get_resolution() / resolution;
    scale = pow(2, int(log2(scale)));    
    resolution_ = sid_.get_resolution() / scale; 

    // 计算sid和shp的最小边界
    cv::Vec4d sid_bound = sid_.get_geo_bound();
    cv::Vec4d shp_bound = shp_.get_geo_bound();
    geo_bound_ = sid_bound;
    if(sid_bound[0]<shp_bound[0])
        geo_bound_[0] = shp_bound[0];
    if(sid_bound[1]<shp_bound[1])
        geo_bound_[1] = shp_bound[1];
    if(sid_bound[2]>shp_bound[2])
        geo_bound_[2] = shp_bound[2];
    if(sid_bound[3]>shp_bound[3])
        geo_bound_[3] = shp_bound[3];

    geo_bound_[0] +=10;
    geo_bound_[1] +=10;
    geo_bound_[2] -=10;
    geo_bound_[3] -=10;
    return true;
}

 // 根据相机的状态（姿态/位置/速度），提取当前相机一段时间内可能观测到的卫星图区域
int Ground::UpdateActiveSatImage(const cv::Rect_<double> &bound){
    sat_geo_bound_ = cv::Vec4d(bound.x, bound.y-/* resolution_ * */bound.height, 
                                    bound.x+resolution_*bound.width, bound.y);
    return sid_.GetROISatImage(bound.x, bound.y, bound.width, bound.height, resolution_, active_sat_img_);
} 
// 根据相机的状态（姿态/位置/速度），提取当前相机一段时间内可能观测到的道路图区域                                 
int Ground::UpdateActiveRoadImage(const cv::Rect_<double> &bound, const int road_width){
    road_geo_bound_ = cv::Vec4d(bound.x, bound.y-/* resolution_ * */bound.height, 
                                    bound.x+/* resolution_* */bound.width, bound.y);
    return shp_.GetROIRoadImage(bound.x, bound.y, bound.width, bound.height, resolution_, road_width, active_road_img_);
}                                   
// 同时运行UpdateActiveSatImage和UpdateActiveRoadImage
int Ground::UpdateActiveSatAndRoadImage(const cv::Rect_<double> &sat_bound, 
                                        const cv::Rect_<double> &road_bound,
                                        const int road_width){
    return UpdateActiveRoadImage(road_bound, road_width)&&UpdateActiveSatImage(sat_bound);
}     

cv::Point2d Ground::SatGeoCorToActiveImageCor(const cv::Point2d &geo_pt){
    double dx = geo_pt.x - sat_geo_bound_[0];
    double dy = road_geo_bound_[3] - geo_pt.y;
    return cv::Point2d(dx/resolution_, dy/resolution_);
}
  
cv::Rect_<double> Ground::CalculateActiveArea(const cv::Point2d &center_pt, 
                                            const double width, 
                                            const double height, 
                                            const cv::Point2d &v){
    cv::Rect_<double> bound;
    bound.x = center_pt.x - width / 2 * resolution_;
    bound.y = center_pt.y + height / 2 * resolution_;

    bound.width = width * resolution_;
    bound.height = height * resolution_;

    // 限制区域不能超过ground的geo_bound_
    if(bound.x<geo_bound_[0])
        bound.x = geo_bound_[0];
    if(bound.y>geo_bound_[3])
        bound.y = geo_bound_[3];
    if(bound.x + bound.width > geo_bound_[2])
        bound.width = geo_bound_[2] - bound.x;
    if(bound.y - bound.height < geo_bound_[1] )
        bound.height = bound.y - geo_bound_[1];
    return bound;
}

void Ground::Print(){
    sid_.Print();
    shp_.Print();
}

void Ground::ShowActiveArea(double scale, int t){
    if(this->active_road_img_.empty()||active_sat_img_.empty()){
        std::cout<<"Image not set!!!"<<std::endl;
    }
    else{
        cv::Mat road_show, sat_show;
        cv::resize(active_road_img_, road_show, cv::Size(0,0), scale, scale);
        cv::resize(active_sat_img_, sat_show, cv::Size(0,0), scale, scale);
        cv::Mat add_img;
        sat_show.copyTo(add_img);
        for(int i=0; i<road_show.rows; i++){
            for(int j=0; j<road_show.cols; j++){
                if(road_show.at<uchar>(i,j) >30)
                    add_img.at<cv::Vec3b> (i,j) = cv::Vec3b(0,0,255);
            }
        }
        cv::imshow("active road on active sat", add_img);
        // cv::imshow("active road", road_show);
        // cv::imshow("active sat", sat_show);
        cv::waitKey(t);
    }
}
    
} // flight_sim
