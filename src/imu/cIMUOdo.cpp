#include "cIMUOdo.h"

cIMUOdo::cIMUOdo(){}

cIMUOdo::~cIMUOdo(){}
  
cIMUOdo::cIMUOdo(const cIMUCorrect &_AccCorrector, const cIMUCorrect &_GroCorrector,  const cIMUState &_IMUState)
{
    myAccCorrector=_AccCorrector;
    myGroCorrector=_GroCorrector;
    myIMUState=_IMUState;
}

  
bool cIMUOdo::setCorrectorFromFile(const std::string &accFilename, const std::string &groFilename)
{
    bool flag1=myAccCorrector.load(accFilename);
    bool flag2=myGroCorrector.load(groFilename);
    return flag1&&flag2;
}
   

void cIMUOdo::setCorrector(const cIMUCorrect &AccCorrector, const cIMUCorrect &GroCorrector)
{
    myAccCorrector=AccCorrector;
    myGroCorrector=GroCorrector;
}


void cIMUOdo::setCorrector(const Eigen::Matrix< double, 3 , 3> &mis_mat_a, const Eigen::Matrix< double, 3 , 1> &scale_vec_a, 
                            const Eigen::Matrix< double, 3 , 1> &bias_vec_a,  const Eigen::Matrix< double, 3 , 3> &mis_mat_g,
                            const Eigen::Matrix< double, 3 , 1> &scale_vec_g, const Eigen::Matrix< double, 3 , 1> &bias_vec_g)
{
    myAccCorrector.setScale(scale_vec_a);
    myAccCorrector.setMisAlignment(mis_mat_a);
    myAccCorrector.setBias(bias_vec_a);
    
    myGroCorrector.setScale(scale_vec_g);
    myGroCorrector.setMisAlignment(mis_mat_g);
    myGroCorrector.setBias(bias_vec_g);
}


void cIMUOdo::setInitGravity(const Eigen::Matrix< double, 3 , 1> &gravity)
{
    myIMUState.setInitGravity(gravity);
}


void cIMUOdo::setInitState(const Eigen::Matrix< double, 10 , 1> & state)
{
    myIMUState.setInitState(state);
}
 

bool cIMUOdo::init(const std::string accFilename, const std::string groFilename,
                const Eigen::Matrix< double, 3 , 1> &gravity,
                const Eigen::Matrix< double, 10 , 1> & state, double time )
{
    if(!this->setCorrectorFromFile(accFilename, groFilename))
        return false;
    this->setInitGravity(gravity);
    this->setInitState(state);
    this->myIMUState.setInitTime(time);
    
    return true;
}


void cIMUOdo::init(const cIMUCorrect &AccCorrector, const cIMUCorrect &GroCorrector,
                const Eigen::Matrix< double, 3 , 1> &gravity,
                const Eigen::Matrix< double, 10 , 1> & state, double time )
{
    this->setCorrector(AccCorrector, GroCorrector);
    this->setInitGravity(gravity);
    this->setInitState(state);
    this->myIMUState.setInitTime(time);
}
 

void cIMUOdo::updateOdo(double time, const Eigen::Matrix< double, 3 , 1> &gro, const Eigen::Matrix< double, 3 , 1> &acc)
{
    Eigen::Matrix< double, 3 , 1> correctedAcc = myAccCorrector.correct(acc);
    Eigen::Matrix< double, 3 , 1> correctedGro = myGroCorrector.correct(gro);
    myIMUState.updataState(time, correctedGro, correctedAcc);
}


void cIMUOdo::getPose(Eigen::Matrix< double, 10, 1> &state)
{
    myIMUState.getState(state);
}

void cIMUOdo::setPose(const Eigen::Matrix< double, 10, 1> &state)
{
    myIMUState.setInitState(state);
}

void cIMUOdo::print()
{
    std::cout<<"/****************Printing Odo state!*****************/"<<std::endl;
    std::cout<<"IMU state:"<<std::endl;
    std::cout<<myIMUState<<std::endl;
    std::cout<<"Acc error model: "<<std::endl;
    std::cout<<myAccCorrector<<std::endl;
    std::cout<<"Gyro error model: "<<std::endl;
    std::cout<<myGroCorrector<<std::endl;
    std::cout<<"/****************Finish*****************/"<<std::endl<<std::endl<<std::endl;
}




