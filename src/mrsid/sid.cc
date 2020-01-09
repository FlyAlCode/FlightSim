#include <math.h>
#include <lt_base.h>
#include <lt_fileSpec.h>
#include <lti_geoCoord.h>
#include <lti_utils.h>
#include <MrSIDImageReader.h>
#include <lti_pixel.h>
#include <lti_sample.h>
#include <lti_scene.h>
#include <lti_sceneBuffer.h>
#include <lt_define.h>
#include "mrsid/sid.h"

// debug
#include <iostream>



LT_USE_NAMESPACE(LizardTech);

namespace flight_sim{

bool MrSid::Init(const std::string & file_name){
	file_name_ = file_name;

	LT_STATUS sts = LT_STS_Uninit;
   
   // make the image reader
   const LTFileSpec fileSpec(file_name_.c_str());
   MrSIDImageReader *reader = MrSIDImageReader::create();
   	if(reader == NULL)
		return false;

   if(reader->initialize(fileSpec)!=LT_STS_Success)
		return false;

   // get some information about the image
   width_ = reader->getWidth();
   height_ = reader->getHeight();
   channels_ = reader->getNumBands();
   data_type_ = reader->getDataType();
   
   min_magnification_ = reader->getMinMagnification();
   max_magnification_ = reader->getMaxMagnification();

   const LTIGeoCoord& geo = reader->getGeoCoord();

   res_[0] = geo.getXRes();
   res_[1] = geo.getYRes();

   bound_[0] = geo.getX();
   bound_[1] = geo.getY() + height_ * res_[1];
   bound_[2] = bound_[0] + width_ * res_[0];
   bound_[3] = geo.getY();

   reader->release();
   reader = NULL;
   
   return true;
}
    
bool MrSid::GetROISatImage(const double x, const double y, 
                     const double width, const double height, 
                     const double resolution, cv::Mat &result_img){
	// 地理坐标系转换到图像坐标系
   // double x_last = x;                                   // (x - bound_[0])/res_[0] ;
	// double y_last = y;                                   // (y - bound_[1])/res_[1] ;
	// double width_last = width;                             // width / res_[0] ;
	// double height_last = height;                             // height / res_[0];
   // 边界检查，如果越界，按边界进行裁剪  
   // if(x_last < bound_[0])
   //    x_last = bound_[0];
   // if(y_last > bound_[1])
   //    y_last = bound_[1];
   // if(x_last + width_last > bound_[0]+bound_[2])
   //    width_last = bound_[0] + bound_[2] - x_last;
   // if(y_last - height_last < bound_[1] - bound_[3])
   //    height_last = y_last - (bound_[1] - bound_[3]);
  
   // if(x < bound_[0])
   //    x_last = bound_[0];
   // if(y > bound_[1])
   //    y_last = bound_[1];
   // if(x + width > bound_[0]+bound_[2])
   //    width_last = bound_[0] + bound_[2] - x;
   // if(y - height < bound_[1] - bound_[3])
   //    height_last = y - (bound_[1] - bound_[3]);

   double x_min, x_max, y_min, y_max;
   if(x>bound_[0])
      x_min = x;
   else 
      x_min = bound_[0];
   if(y-height>bound_[1])
      y_min = y-height;
   else 
      y_min = bound_[1];
   if(x+width<bound_[2])
      x_max = x+width;
   else 
      x_max = bound_[2];
   if(y<bound_[3])
      y_max = y;
   else 
      y_max = bound_[3];

   double x_last = x_min;
   double y_last = y_max;
   double width_last = x_max - x_min;
   double height_last = y_max - y_min;

   if(width_last<=0 || height_last<=0 || resolution<=0)
		return false;
   // if(x_last<0)
	// 	x_last = 0;
	// if(y_last <0)
	// 	y_last = 0;
	// if(x_last + width_last>width_-1)
	// 	width_last = width_ - 1 - x_last;
	// if(y_last + height_last > height_ -1)
	// 	height_last = height_ - 1 - y_last;

   // decode the whole image at given resolution
   double scale = res_[0]/resolution;
   scale = pow(2, int(log2(scale)));                        // 由于sid的库只支持2的倍数的缩放，因此这里将尺度转换到最近的2的倍数

   x_last = (x_last - bound_[0])/res_[0] * scale;
   y_last = (y_last - bound_[3])/res_[1] * scale;
   width_last = width_last / res_[0] * scale;
   height_last = height_last / res_[0] * scale;
    // debug
   // std::cout<<"sid active image area:";
   // std::cout<<x_last<<" "<<y_last<<" "<<width_last<<" "<<height_last<<std::endl;

   // x_last *= scale;
   // y_last *= scale;
   // width_last *= scale;
   // height_last *= scale;

   // debug
   // std::cout<<"last scale = "<<scale<<std::endl;
   // std::cout<<"x_last = "<<x_last<<std::endl;
   // std::cout<<"y_last = "<<y_last<<std::endl;
   // std::cout<<"width_last = "<<width_last<<std::endl;
   // std::cout<<"height_last = "<<height_last<<std::endl;

	LT_STATUS sts = LT_STS_Uninit;

   // make the image reader
   const LTFileSpec fileSpec(file_name_.c_str());
   MrSIDImageReader *reader = MrSIDImageReader::create();
   if(reader == NULL)
		return false;

   if(reader->initialize(fileSpec)!=LT_STS_Success)
		return false;

   const LTIScene scene(x_last, y_last, width_last, height_last, scale);
   std::cout<<"scene info: "<<scene.getNumRows()<<" "<<scene.getNumCols()<<std::endl;
   
   // construct the buffer we're decoding into
   // note we choose to allocate our own buffer, rather than let
   // LTISceneBuffer implicitly allocate one for us   
   lt_uint16 numBands = reader->getNumBands();
   LTIDataType datatype = reader->getDataType();
   
   const lt_uint32 bytesPerSample = LTIUtils::getNumBytes(datatype);
   const lt_uint32 numPixels = scene.getNumCols() * scene.getNumRows();
   const lt_uint32 bytesPerBands = numPixels * bytesPerSample;
   
   lt_uint8 **bsqData = new lt_uint8 *[numBands];
   for(lt_uint16 i = 0; i < numBands; i++)
      bsqData[i] = new lt_uint8[bytesPerBands];
   
   LTISceneBuffer sceneBuffer(reader->getPixelProps(),
                              scene.getNumCols(),
                              scene.getNumRows(),
                              reinterpret_cast<void **>(bsqData));
 
   // only let the decoder use the calling thread 
   // otherwise it will try to uses all the cores
   reader->setMaxWorkerThreads(4);
 
   // perform the decode
   if(reader->read(scene, sceneBuffer)!=LT_STS_Success){
	    // clean up
   		for(lt_uint16 i = 0; i < numBands; i++)
      		delete [] bsqData[i];
   		delete [] bsqData;
		return false;
   }	
	
   // create OpenCV mat
   int mat_data_type;
   if(channels_ == 1){
	   	switch (datatype) {
	   		case LTI_DATATYPE_FLOAT32:
				mat_data_type = CV_32FC1;
		   		break;
			case LTI_DATATYPE_FLOAT64: 	
	   			mat_data_type = CV_64FC1;
				break;
	   		default:
			   	mat_data_type = CV_8UC1;
		   		break;
	   }
   }
   if(channels_ == 3){
	   switch (datatype) {
	   		case LTI_DATATYPE_FLOAT32:
				mat_data_type = CV_32FC3;
		   		break;
			case LTI_DATATYPE_FLOAT64: 	
	   			mat_data_type = CV_64FC3;
				break;
	   		default:
			   	mat_data_type = CV_8UC3;
		   		break;
	   }
   }
   // allocate OpenCV mat memory
   // uchar *mat_data = new uchar[numBands * bytesPerBands];
   void * mat_data = new uchar[numBands * bytesPerBands];
	// for(int i=0; i<bytesPerBands; i+=bytesPerSample){
	// 	for(int j=0; j<numBands; j++){
	// 		memcpy(mat_data+i*numBands + j, *(bsqData+j)+i, bytesPerSample);
	// 		// debug
	// 		// std::cout<<int(*(*(bsqData+j)+i))<<std::endl;
	// 	}
	// }
   sceneBuffer.exportDataBIP(mat_data);

   cv::Mat tmp(height_last, width_last, mat_data_type, mat_data);
   tmp.copyTo(result_img);  

   // clean up
   for(lt_uint16 i = 0; i < numBands; i++)
      delete [] bsqData[i];
   delete [] bsqData;   

   return true;
}

void MrSid::Print(){
	std::cout<<"\n\nSomething about the sid file"<<std::endl;
	std::cout<<"width = "<<width_<<std::endl;
	std::cout<<"height = "<<height_<<std::endl;
	std::cout<<"channels = "<<channels_<<std::endl;
	std::cout<<"min_magnification = "<<min_magnification_<<std::endl;
	std::cout<<"max_magnification = "<<max_magnification_<<std::endl;
	std::cout<<"res_x = "<<res_[0]<<std::endl;
	std::cout<<"res_y = "<<res_[1]<<std::endl;
   std::cout<<"geo_bound = "<<bound_[0]<<" "<<bound_[1]<<" "<<bound_[2]<<" "<<bound_[3]<<std::endl;
}

    
} // namespace flight_sim