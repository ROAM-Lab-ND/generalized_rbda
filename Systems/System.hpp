#ifndef __SYSTEM_H__
#define __SYSTEM_H__

/*!
 * @file System.hpp
 * @brief System includes
 * 1) Robot dynamic model
 * 2) Controller: MPC, WBC, Backflip, ...
 * 3) State Machine
 * 4) State estimator
 */

#include <stdio.h>
#include <common/dynamics/Robot.h>
#include <SimUtilities/VisualizationData.h>

template <typename T>
class System{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    System(bool is_sim){ _is_sim = is_sim; }
    virtual ~System(){}
    virtual bool initialization() = 0;
    virtual void onestep_forward() = 0;
    
    virtual void onestep_forward_train() = 0;
    bool _trainingComplete = false;

    virtual void renderSystem() = 0 ;
    virtual void clearVisualization()=0;
    virtual bool Estop() = 0;
    virtual void updateFBModelStateEstimate() = 0;

    const Robot<T> * getRobot(){ return _robot; }
    void setFBModelState(const FBModelState<T> & state){
      _state = state;
    }

    T getOnestepUsec(){
      return _update_dt*1.e6;
    }

    virtual void onestep_forward_test(){}

  protected:
    T _update_dt;
    Robot<T>* _robot = nullptr;
    FBModelState<T> _state;
    bool _is_sim = true;


};

#endif
