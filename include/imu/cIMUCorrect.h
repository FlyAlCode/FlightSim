#include <Eigen/Core>
#include <iostream>
#include <fstream>


/** @brief This object contains the calibration parameters (misalignment, scale factors, ...)
 *         of a generic orthogonal sensor triad (accelerometers, gyroscopes, etc.)
 * 
 * Triad model:
 *         
 * -Misalignment matrix:
 * 
 * general case:
 * 
 *     [    1     -mis_yz   mis_zy  ]
 * T = [  mis_xz     1     -mis_zx  ]
 *     [ -mis_xy   mis_yx     1     ]
 * 
 * "body" frame spacial case:
 * 
 *     [  1     -mis_yz   mis_zy  ]
 * T = [  0        1     -mis_zx  ]
 *     [  0        0        1     ]
 * 
 * Scale matrix:
 * 
 *     [  s_x      0        0  ]
 * K = [   0      s_y       0  ]
 *     [   0       0       s_z ]
 * 
 * Bias vector:
 * 
 *     [ b_x ]
 * B = [ b_y ]
 *     [ b_z ]
 * 
 * Given a raw sensor reading X (e.g., the acceleration ), the calibrated "unbiased" reading X' is obtained
 * 
 * X' = T*K*(X - B)
 * 
 * with B the bias (variable) + offset (constant, possibbly 0), or, equivalently:
 * 
 * X' = T*K*X - B'
 * 
 * with B' = T*K*B
 * 
 * Without knowing the value of the bias (and with offset == 0), the calibrated reading X'' is simply:
 * 
 * X'' = T*K*X
*/
 class cIMUCorrect
{
public:
  /** @brief Basic "default" constructor: without any parameter, it initilizes the calibration parameter with 
   *         default values (zero scaling factors and biases, identity misalignment matrix)
   */
  cIMUCorrect( const double &mis_yz = double(0), const double &mis_zy = double(0), const double &mis_zx = double(0), 
                    const double &mis_xz = double(0), const double &mis_xy = double(0), const double &mis_yx = double(0), 
                    const double &s_x = double(1),    const double &s_y = double(1),    const double &s_z = double(1), 
                    const double &b_x = double(0),    const double &b_y = double(0),    const double &b_z  = double(0) );
 
  ~cIMUCorrect(){}
               
  inline double misYZ() const { return -mis_mat_(0,1); };
  inline double misZY() const { return mis_mat_(0,2); };
  inline double misZX() const { return -mis_mat_(1,2); };
  inline double misXZ() const { return mis_mat_(1,0); };
  inline double misXY() const { return -mis_mat_(2,0); };
  inline double misYX() const { return mis_mat_(2,1); };

  inline double scaleX() const { return scale_mat_(0,0); };
  inline double scaleY() const { return scale_mat_(1,1); };
  inline double scaleZ() const { return scale_mat_(2,2); };
      
  inline double biasX() const { return bias_vec_(0); };
  inline double biasY() const { return bias_vec_(1); };
  inline double biasZ() const { return bias_vec_(2); };
  
  inline const Eigen::Matrix< double, 3 , 3>& getMisalignmentMatrix() const { return mis_mat_; };
  inline const Eigen::Matrix< double, 3 , 3>& getScaleMatrix() const { return scale_mat_; };
  inline const Eigen::Matrix< double, 3 , 1>& getBiasVector() const { return bias_vec_; };
    
  inline void setScale( const Eigen::Matrix< double, 3 , 1> &s_vec ) 
  { 
    scale_mat_(0,0) = s_vec(0); scale_mat_(1,1) = s_vec(1);  scale_mat_(2,2) = s_vec(2); 
    update();
  };
  
  inline void setMisAlignment( const Eigen::Matrix< double, 3 , 3> &mis_mat ) 
  {
      mis_mat_=mis_mat;
      update();
  }

  inline void setBias( const Eigen::Matrix< double, 3 , 1> &b_vec ) 
  { 
    bias_vec_ = b_vec;
    update();
  };
  
  /** @brief Load the calibration parameters from a simple text file.
   * 
   * The file should containts a sequence of two, space separated 3X3 matrixes 
   * (the misalignment and the scale matrix) followed by a 3x1 biases vector (see the load()
   * function)
   */
  bool load( std::string filename );
  
  /** @brief Save the calibration parameters in a simple text file.
   * 
   * The file will containts a sequence of two, space separated 3X3 matrixes 
   * (the misalignment and the scale matrix) followed by a 3x1 biases vector 
   */
  bool save( std::string filename ) const;

  /** @brief Normalize a raw data X by correcting the misalignment and the scale,
   *         i.e., by applying the equation  X'' = T*K*X
   */
  inline Eigen::Matrix< double, 3 , 1> normalize( const Eigen::Matrix< double, 3 , 1> &raw_data ) const
  {
    return ms_mat_*raw_data;
  };
  
  /** @brief Normalize a raw data X by removing the biases and 
   *         correcting the misalignment and the scale, 
   *         i.e., by applying the equation  X' = T*K*(X - B)
   */
  inline Eigen::Matrix< double, 3 , 1> unbiasNormalize( const Eigen::Matrix< double, 3 , 1> &raw_data ) const
  {
    return ms_mat_*(raw_data - bias_vec_); 
  };
  
   inline Eigen::Matrix< double, 3 , 1> correct( const Eigen::Matrix< double, 3 , 1> &raw_data ) const
  {
    return ms_mat_*(raw_data - bias_vec_ + variable_bias); 
  };
  
  inline void correct( double *data )
  {
      Eigen::Matrix< double, 3 , 1> raw_data;
      raw_data<<data[0], data[1], data[2];
      Eigen::Matrix< double, 3 , 1> correct_data=this->correct(raw_data);
      
     data[0]=correct_data(0);
     data[1]=correct_data(1);
     data[2]=correct_data(2);
  };

  /** @brief Remove the biases from a raw data */
  inline Eigen::Matrix< double, 3 , 1> unbias( const Eigen::Matrix< double, 3 , 1> &raw_data ) const
  {
    return raw_data - bias_vec_; 
  };
  
  
  
private:

  /** @brief Update internal data (e.g., compute Misalignment * scale matrix) 
   *         after a parameter is changed */
  void update();
  
  /** @brief Misalignment matrix */
  Eigen::Matrix< double, 3 , 3> mis_mat_;
  /** @brief Scale matrix */
  Eigen::Matrix< double, 3 , 3> scale_mat_;
  /** @brief Bias vector */
  Eigen::Matrix< double, 3 , 1> bias_vec_;
  /** @brief Misalignment * scale matrix */
  Eigen::Matrix< double, 3 , 3> ms_mat_;
  
    /** @brief variable bais vector(estimate online to enable more wide use) */
  Eigen::Matrix< double, 3 , 1> variable_bias;
};


std::ostream& operator<<(std::ostream& os, const cIMUCorrect& calibdoubleriad);

//typedef cIMUCorrect<double> cIMUCorrect;


//#include "../../lib/cIMUCorrect.cpp"
