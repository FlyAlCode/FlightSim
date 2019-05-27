#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>



class cIMUState
{
public:
    cIMUState(Eigen::Matrix< double, 3, 1> _gravity,
                Eigen::Matrix< double, 10, 1> _initState,
                Eigen::Matrix< double, 10, 1> _currentState, double _currentTime);
    cIMUState();
    ~cIMUState();
    void setInitState(double *state);    //set the init pose, (attitude,velocity, position)
    void setInitState(const Eigen::Matrix< double, 10, 1> &state);
    void setInitTime(double time);
    void setInitGravity(double *gravity);//set the init gravity and the navigation coordination meanwhile
    void setInitGravity(const Eigen::Matrix< double, 3, 1> &_gravity);
    void updataState(double time, const Eigen::Matrix< double, 3, 1> &gro, const Eigen::Matrix< double, 3, 1> &acc);
    void getState(Eigen::Matrix< double, 10, 1> &state);
    Eigen::Matrix< double, 10, 1> getState() const;
    Eigen::Matrix< double, 3, 1> getGravity() const;
    double getTime() const;
    
private:
    void calDstate(const Eigen::Matrix< double, 10, 1> &dstate, const Eigen::Matrix< double, 3, 1> &gro, 
                                 const Eigen::Matrix< double, 3, 1> &acc, Eigen::Matrix< double, 10, 1> &state);
    Eigen::Matrix< double, 3, 1> gravity;
    Eigen::Matrix< double, 10, 1> initState;
    Eigen::Matrix< double, 10, 1> currentState;
    double currentTime;//current time, changed after all update is done
    
};

std::ostream& operator<<(std::ostream& os, const cIMUState &imuState);

//typedef cIMUState<double> cIMUState;

//#include "../../lib/cIMUCorrect"
