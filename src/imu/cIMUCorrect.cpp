#include "cIMUCorrect.h"



 
cIMUCorrect::cIMUCorrect( const double &mis_yz, const double &mis_zy, const double &mis_zx, 
                                                const double &mis_xz, const double &mis_xy, const double &mis_yx, 
                                                const double &s_x, const double &s_y, const double &s_z, 
                                                const double &b_x, const double &b_y, const double &b_z )
{
  mis_mat_ <<  double(1)   , -mis_yz  ,  mis_zy  ,
                mis_xz ,  double(1)   , -mis_zx  ,  
               -mis_xy ,  mis_yx  ,  double(1)   ;
              
  scale_mat_ <<   s_x  ,   double(0)  ,  double(0) ,
                 double(0) ,    s_y   ,  double(0) ,  
                 double(0) ,   double(0)  ,   s_z  ;
                    
  bias_vec_ <<  b_x , b_y , b_z ; 
  
  variable_bias << 0, 0, 0;
  update();
}

 
  bool cIMUCorrect::load( std::string filename )
{
  std::ifstream file( filename.c_str() );
  if (file.is_open())
  {
    double mat[9] = {0};
    
    for( int i=0; i<9; i++)
      file >> mat[i];

    mis_mat_ = Eigen::Map< const Eigen::Matrix< double, 3, 3, Eigen::RowMajor> >(mat);
      
    for( int i=0; i<9; i++)
      file >> mat[i];
    
    scale_mat_ = Eigen::Map< const Eigen::Matrix< double, 3, 3, Eigen::RowMajor> >(mat);
        
    for( int i=0; i<3; i++)
      file >> mat[i];
    
    bias_vec_ = Eigen::Map< const Eigen::Matrix< double, 3, 1> >(mat);    
    
    update();
    
    return true;
  }
  std::cout<<"Can't find imu correct model file!"<<std::endl;
  return false;  
}

 
  bool cIMUCorrect::save( std::string filename ) const
{
  std::ofstream file( filename.data() );
  if (file.is_open())
  {
    file<<mis_mat_<<std::endl<<std::endl
        <<scale_mat_<<std::endl<<std::endl
        <<bias_vec_<<std::endl<<std::endl;
    
    return true;
  }
  return false;  
}

 void cIMUCorrect::update()
{
  ms_mat_ = mis_mat_*scale_mat_;
}


std::ostream& operator<<(std::ostream& os, const cIMUCorrect& calibdoubleriad)
{
  os<<"Misalignment Matrix"<<std::endl;
  os<<calibdoubleriad.getMisalignmentMatrix()<<std::endl;
  os<<"Scale Matrix"<<std::endl;
  os<<calibdoubleriad.getScaleMatrix()<<std::endl;
  os<<"Bias Vector"<<std::endl;
  os<<calibdoubleriad.getBiasVector()<<std::endl;
  return os;
}


