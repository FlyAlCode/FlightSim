#ifndef FLIGHT_SIM_SHP_H_
#define FLIGHT_SIM_SHP_H_

#include <opencv2/core/core.hpp>

namespace flight_sim{
class SHP{
public:
    bool Init(const std::string &file_name);
    bool GetROIRoadImage(const double x,                    // 地理坐标,x,y单位为m
                        const double y, 
                        const double width, 
                        const double height, 
                        const double scale,                 // 分辨率，为1时，表示分辨率为1m/pixel，为3时，表示分辨率为1/3m/pixel
                        const double road_width,            // 画矢量图时的线宽
                        cv::Mat &img);
    void Print();

    inline cv::Vec4d get_geo_bound() {return cv::Vec4d(geo_bound_[0], geo_bound_[1], geo_bound_[2], geo_bound_[3]);}
private:
    void ShpPtsToPixels(const std::vector<std::vector<cv::Point2d> > &shp_pts,
                        const double scale,
                        std::vector<std::vector<cv::Point> >&pixels);

    std::string coordination_;
    // cv::Rect_<double> geo_bound_;                               // left-down-x left-down-y width height
    double geo_bound_[4];                                           // minx, miny, maxx, maxy
    // cv::Rect pixel_bound_;
    std::vector< std::vector<cv::Point2d> > elements_;
}; 
} // namespace flight_sim



#endif