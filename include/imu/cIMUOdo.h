#include "cIMUCorrect.h"
#include "cIMUState.h"


class cIMUOdo {
public:
    cIMUOdo();
    ~cIMUOdo();
    
    cIMUOdo(const cIMUCorrect &_AccCorrector, const cIMUCorrect &_GroCorrector,  const cIMUState &_IMUState);
    bool setCorrectorFromFile(const std::string &accFilename, const std::string &groFilename);
    void setCorrector(const cIMUCorrect &AccCorrector, const cIMUCorrect &GroCorrector);
    void setCorrector(const Eigen::Matrix< double, 3 , 3> &mis_mat_a, const Eigen::Matrix< double, 3 , 1> &scale_vec_a, const Eigen::Matrix< double, 3 , 1> &bias_vec_a, 
                      const Eigen::Matrix< double, 3 , 3> &mis_mat_g, const Eigen::Matrix< double, 3 , 1> &scale_vec_g, const Eigen::Matrix< double, 3 , 1> &bias_vec_g);
    
    void setInitGravity(const Eigen::Matrix< double, 3 , 1> &gravity);
    void setInitState(const Eigen::Matrix< double, 10 , 1> & state);
    
    bool init(const std::string accFilename, const std::string groFilename,
                const Eigen::Matrix< double, 3 , 1> &gravity,
                const Eigen::Matrix< double, 10 , 1> & state, double time );
    void init(const cIMUCorrect &AccCorrector, const cIMUCorrect &GroCorrector,
                const Eigen::Matrix< double, 3 , 1> &gravity,
                const Eigen::Matrix< double, 10 , 1> & state, double time );
    
    void updateOdo(double time, const Eigen::Matrix< double, 3 , 1> &gro, const Eigen::Matrix< double, 3 , 1> &acc);
    
    void getPose(Eigen::Matrix< double, 10, 1> &state);
    void setPose(const Eigen::Matrix< double, 10, 1> &state);
    void print();
    
        
private:
    cIMUCorrect myAccCorrector;
    cIMUCorrect myGroCorrector;
    cIMUState myIMUState;
};



//typedef cIMUOdo<double> cIMUOdo;

//#include "../../lib/cIMUOdo"
