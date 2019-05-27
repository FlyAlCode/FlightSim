#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <shapefil.h>
#include "shp/shp.h"

namespace flight_sim {

/* 1，读取边界信息
 * 2，读取所有的折线
 */
bool SHP::Init(const std::string &shp_file_name){
    SHPHandle	hSHP;
    int		nShapeType, nEntities, i, iPart, bValidate = 0,nInvalidCount=0;
    int         bHeaderOnly = 0;
    const char 	*pszPlus;
    double 	adfMinBound[4], adfMaxBound[4];
    int nPrecision = 15;
    
    elements_.clear();

    /* -------------------------------------------------------------------- */
    /*      Open the passed shapefile.                                      */
    /* -------------------------------------------------------------------- */
    hSHP = SHPOpen( shp_file_name.c_str(), "rb" );

    if( hSHP == NULL )  {
        printf( "Unable to open:%s\n", shp_file_name.c_str() );
        return false;
    }

    /* -------------------------------------------------------------------- */
    /*      Print out the file bounds.                                      */
    /* -------------------------------------------------------------------- */
    SHPGetInfo( hSHP, &nEntities, &nShapeType, adfMinBound, adfMaxBound );
    
    // geo_bound_.x = adfMinBound[0];
    // geo_bound_.y = adfMinBound[1];
    // geo_bound_.width = adfMaxBound[0] - adfMinBound[0];
    // geo_bound_.height = adfMaxBound[1] - adfMinBound[1];
    geo_bound_[0] = adfMinBound[0];
    geo_bound_[1] = adfMinBound[1];
    geo_bound_[2] = adfMaxBound[0];
    geo_bound_[3] = adfMaxBound[1];

    /* -------------------------------------------------------------------- */
    /*	Skim over the list of shapes, printing all the vertices.            */
    /* -------------------------------------------------------------------- */
    for( i = 0; i < nEntities; i++ )
    {
        int		j;
        SHPObject	*psShape;

        psShape = SHPReadObject( hSHP, i );

        if( psShape == NULL ) {
            fprintf( stderr,
                     "Unable to read shape %d, terminating object reading.\n",
                    i );
            break;
        }
        
        if(psShape->nSHPType ==3||psShape->nSHPType ==13||psShape->nSHPType ==23) {     // current we only deal with polyLine
            if( psShape->nParts > 0 && psShape->panPartStart[0] != 0 ) {
                fprintf( stderr, "panPartStart[0] = %d, not zero as expected.\n",
                         psShape->panPartStart[0] );
            }
            
            std::vector<cv::Point2d> part_pt_tmp;
            for( j = 0, iPart = 1; j < psShape->nVertices; j++ ) {
                if( iPart < psShape->nParts && psShape->panPartStart[iPart] == j ) {                // start a new parts, a part for a element
                        elements_.push_back(part_pt_tmp);
                        part_pt_tmp.clear();
                        iPart++;
                }
                part_pt_tmp.push_back(cv::Point2d(psShape->padfX[j], psShape->padfY[j]));
            }
            elements_.push_back(part_pt_tmp);
        }
 
        SHPDestroyObject( psShape );
    }

    SHPClose( hSHP );
  
#ifdef USE_DBMALLOC
    malloc_dump(2);
#endif  
    return true;
}

bool SHP::GetROIRoadImage(const double x, 
                        const double y, 
                        const double width, 
                        const double height, 
                        const double scale, 
                        const double road_width,
                        cv::Mat &result_img){
    // 边界检查
    // double x_last = x;
    // double y_last = y;
    // double width_last = width;
    // double height_last = height;
    // if(x_last<geo_bound_.x)
    //     x_last = geo_bound_.x;
    // if(y_last>geo_bound_.y+geo_bound_.height)
    //     y_last = geo_bound_.y+geo_bound_.height;
    // if(x_last+width_last>geo_bound_.x+geo_bound_.width)
    //     width_last = geo_bound_.x+geo_bound_.width-x_last;
    // if(y_last - height_last<geo_bound_.y)
    //     height_last = y_last - geo_bound_.y;
    // if(x<geo_bound_.x)
    //     x_last = geo_bound_.x;
    // if(y>geo_bound_.y+geo_bound_.height)
    //     y_last = geo_bound_.y+geo_bound_.height;
    // if(x+width>geo_bound_.x+geo_bound_.width)
    //     width_last = geo_bound_.x+geo_bound_.width-x;
    // if(y - height<geo_bound_.y)
    //     height_last = y - geo_bound_.y;

    double x_min, x_max, y_min, y_max;
   if(x>geo_bound_[0])
      x_min = x;
   else 
      x_min = geo_bound_[0];
   if(y-height>geo_bound_[1])
      y_min = y-height;
   else 
      y_min = geo_bound_[1];
   if(x+width<geo_bound_[2])
      x_max = x+width;
   else 
      x_max = geo_bound_[2];
   if(y<geo_bound_[3])
      y_max = y;
   else 
      y_max = geo_bound_[3];

   double x_last = x_min;
   double y_last = y_max;
   double width_last = x_max - x_min;
   double height_last = y_max - y_min;
    if(width_last<=0 || height_last<=0 || scale<=0)
        return false;
    // cv::Rect pixel_bound;
    // pixel_bound.x = 0;
    // pixel_bound.y = 0;
    // pixel_bound.width = geo_bound_.width/scale;
    // pixel_bound.height = geo_bound_.height/scale;
    // if(start_pts.x<0)
    //     start_pts.x = 0;
    // if(start_pts.y < 0)
    //     start_pts.y = 0;
    // if(end_pts.x>pixel_bound.width-1)
    //     end_pts.x  = pixel_bound.width -1;
    // if(end_pts.y>pixel_bound.height -1)
    //     end_pts.y = pixel_bound.height -1;
    std::vector<cv::Point2d> area_corners;
    area_corners.push_back(cv::Point2d(x_last, y_last));
    std::vector<std::vector<cv::Point2d> > area_pts;
    area_pts.push_back(area_corners);
    std::vector<std::vector<cv::Point> > area_pixels;
    ShpPtsToPixels(area_pts, scale, area_pixels);
    
    cv::Point start_pts(area_pixels[0][0]);
    cv::Point end_pts(start_pts.x + width_last/scale, start_pts.y + height_last/scale);

    // if(end_pts.x<start_pts.x || end_pts.y<start_pts.y)
    //     return false;

     // debug
    std::cout<<"shp active image area: ";
    std::cout<<start_pts.x<<" "<<start_pts.y<<" "<<width_last/scale<<" "<<height_last/scale<<std::endl;

    cv::Mat img = cv::Mat::zeros( end_pts.y - start_pts.y,
                                  end_pts.x - start_pts.x, CV_8U);
    
    cv::Point offset =  - start_pts;
    cv::Rect bound(0, 0, img.cols, img.rows);
    cv::Point line_start, line_end;
    bool start_set;
    cv::Point tmp;
    std::vector<std::vector<cv::Point> > pixels;
    ShpPtsToPixels(elements_, scale, pixels);
    for(int i=0; i<pixels.size(); ++i){
        start_set = false;
        for(int j=0; j<pixels[i].size(); ++j){
            tmp = pixels[i][j] + offset;
            if(!start_set){
                line_start = tmp;
                start_set = true;
            }   
            else{
                line_end = tmp;
                cv::line(img, line_start, line_end, cv::Scalar(255), road_width);               // line out of the img will not be drawed
                line_start = line_end;
            }
        }
    }
    img.copyTo(result_img);
}

void SHP::Print(){
    std::cout<<"shp geo_bound = "<<geo_bound_<<std::endl;
    // std::cout<<
}

void SHP::ShpPtsToPixels(const std::vector<std::vector<cv::Point2d> > &shp_pts,
                        const double scale,
                        std::vector<std::vector<cv::Point> >&pixels){
    pixels.clear();

    int whole_img_width = (geo_bound_[2] - geo_bound_[0]) / scale;
    int whole_img_height = (geo_bound_[3] - geo_bound_[1]) / scale;
    
    cv::Point pixel_tmp;
    std::vector<cv::Point> pixels_tmp;
    for(int i=0; i<shp_pts.size(); ++i){
        pixels_tmp.clear();
        for(int j=0; j<shp_pts[i].size(); ++j){
            pixel_tmp.x = (shp_pts[i][j].x - geo_bound_[0]) / scale;
            pixel_tmp.y = (geo_bound_[3] - shp_pts[i][j].y)/scale;
            pixels_tmp.push_back(pixel_tmp);
        }
        pixels.push_back(pixels_tmp);
    }
}
} // namespace flight_sim 