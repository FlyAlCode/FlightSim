#include "cIMUState.h"



/**************implement*******************/
cIMUState::cIMUState(){}

cIMUState::~cIMUState(){}

cIMUState::cIMUState(Eigen::Matrix< double, 3, 1> _gravity,
                Eigen::Matrix< double, 10, 1> _initState,
                Eigen::Matrix< double, 10, 1> _currentState, double _currentTime)
{
    gravity=_gravity;
    initState=_initState;
    currentState=_currentState;
    currentTime=_currentTime;
}


void cIMUState::setInitTime(double time)
{
    currentTime=time;
}


void cIMUState::setInitState(double *state)
{
    initState= Eigen::Map<const Eigen::Matrix< double, 10, 1> >(state) ;
    currentState=initState;
}


void cIMUState::setInitState(const Eigen::Matrix< double, 10, 1> &state)
{
    initState= state;
    currentState=initState;
}


void cIMUState::setInitGravity(double *_gravity)     //set gravity in init coordination
{
    gravity = Eigen::Map<const Eigen::Matrix< double, 3, 1> >(_gravity);
}


void cIMUState::setInitGravity(const Eigen::Matrix< double, 3, 1> &_gravity)
{
    gravity = _gravity;
}



void cIMUState::calDstate(const Eigen::Matrix< double, 10, 1> &state, const Eigen::Matrix< double, 3, 1> &gro, 
                                 const Eigen::Matrix< double, 3, 1> &acc, Eigen::Matrix< double, 10, 1> &dstate)
{
    // Quaternion Equations of Motion
    
    Eigen::Quaternion<double> tmp = Eigen::Quaternion<double>(state(0),state(1),state(2),state(3))*Eigen::Quaternion<double>(0,gro(0),gro(1),gro(2));
    dstate(0) = 0.5 * tmp.w();
    dstate(1) = 0.5 * tmp.x();
    dstate(2) = 0.5 * tmp.y();
    dstate(3) = 0.5 * tmp.z();
    // Position Equations of Motion
    dstate.segment(4,3) = Eigen::Quaternion<double>(state(0),state(1),state(2),state(3)).matrix()*acc - gravity;
    // Velocity Equations of Motion, the influence of rotation is not considered.
    dstate.tail(3) = state.segment(4,3);
    /*------ End Equations of Motion ------*/
}


void cIMUState::updataState(double time, const Eigen::Matrix< double, 3, 1> &gro, const Eigen::Matrix< double, 3, 1> &acc)
{
    double dt=time-currentTime;
    Eigen::Matrix< double, 10, 1> k_1;
    Eigen::Matrix< double, 10, 1> k_2;
    Eigen::Matrix< double, 10, 1> k_3;
    Eigen::Matrix< double, 10, 1> k_4;
    
    calDstate(currentState, gro, acc, k_1);
    calDstate(currentState+0.5*dt*k_1, gro, acc, k_2);
    calDstate(currentState+0.5*dt*k_2, gro, acc,k_3 );
    calDstate(currentState+dt*k_3, gro, acc, k_4);

    currentState +=  (dt/6.0)*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4);

    
    Eigen::Quaternion<double> Q(currentState(0),currentState(1),currentState(2),currentState(3));
    Q.normalize();
    currentState(0)=Q.w();
    currentState(1)=Q.x();
    currentState(2)=Q.y();
    currentState(3)=Q.z();
    
    currentTime=time;
}


void cIMUState::getState(Eigen::Matrix< double, 10, 1> &state)
{
    state= this->currentState;
    //std::cout<<"[cIMUState]: state= "<<state<<std::endl;
}

Eigen::Matrix< double, 10, 1> cIMUState::getState() const
{
    return this->currentState;
}

Eigen::Matrix< double, 3, 1> cIMUState::getGravity() const
{
    return this->gravity;
}

double cIMUState::getTime() const
{
    return this->currentTime;
}

std::ostream& operator<<(std::ostream& os, const cIMUState &imuState)
{
    os<<"time"<<std::endl;
    os<<imuState.getTime()<<std::endl;
    os<<"gravity "<<std::endl;
    os<<imuState.getGravity()<<std::endl;
    os<<"current state"<<std::endl;
    os<<imuState.getState()<<std::endl;
    return os;
}




