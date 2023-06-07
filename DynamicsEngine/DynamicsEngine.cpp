/*! @file DynamicsEngine.cpp
 *  @brief Rigid Body Dynamics Simulator with Collisions
 *
 *  Combines ABA, Collisions, integrator, and any other external forces to run a
 * simulation. Doesn't do any graphics.
 */

#include "DynamicsEngine.h"
#include "Collision/ContactImpulse.h"
#include "Collision/ContactSpringDamper.h"
#include <utilities/pretty_print.h>

/*!
 * Initialize the dynamics simulator by allocating memory for ABA matrices
 */
template <typename T>
DynamicsEngine<T>::DynamicsEngine(FloatingBaseModel<T> * model,
                                        bool useSpringDamper)
    : _model(model), _useSpringDamper(useSpringDamper) {
  if (_useSpringDamper) {
    _contact_constr = new ContactSpringDamper<T>(_model);
  } else {
    _contact_constr = new ContactImpulse<T>(_model);
  }

  _state.bodyVelocity = SVec<T>::Zero();
  _state.bodyPosition = Vec3<T>::Zero();
  _state.bodyOrientation = Quat<T>::Zero();
  _state.q = DVec<T>::Zero(_model->_nDof - 6);
  _state.qd = DVec<T>::Zero(_model->_nDof - 6);

  _dstate.dBodyPosition= Vec3<T>::Zero();
  _dstate.dBodyVelocity = SVec<T>::Zero();
  _dstate.qdd = DVec<T>::Zero(_model->_nDof - 6);

  _lastBodyVelocity.setZero();
  bodyAcc.setZero();bodyAcc1.setZero();bodyAcc2.setZero();
  bodyVel.setZero();bodyVel1.setZero();bodyVel2.setZero();

  // Homing initilization
  _homing.active_flag = false;
}

/*!
 * Take one simulation step
 * @param dt : timestep duration
 * @param tau : joint torques
 */
template <typename T>
void DynamicsEngine<T>::step(T dt, const DVec<T> &tau, T kp, T kd ) {
  // fwd-kin on gc points
  // compute ground contact forces
  // aba
  // integrate
  forwardKinematics();           // compute forward kinematics
  updateCollisions(dt, kp, kd);  // process collisions
  // Process Homing
  if(_homing.active_flag) {
    Mat3<T> R10_des = rpyToRotMat(_homing.rpy);              // R10_des
    Mat3<T> R10_act = _model->getOrientation(5).transpose();  // R10
    Mat3<T> eR01 = R10_des.transpose()*R10_act;              // eR * R01 = R01_des
    
    Vec4<T> equat = rotationMatrixToQuaternion(eR01.transpose());
    Vec3<T> angle_axis = quatToso3(equat); // in world frame

    Vec3<T> p = _model->getPosition(5);
    Vec3<T> f = _homing.kp_lin*(_homing.position - p)-_homing.kd_lin*_model->getLinearVelocity(5);

    // Note: External forces are spatial forces in the {0} frame. 
    _model->_externalForces.at(5) += forceToSpatialForce(f,p);
    _model->_externalForces.at(5).head(3) += _homing.kp_ang*angle_axis - _homing.kd_ang*_model->getAngularVelocity(5);
  }
  runABA(tau);                   // dynamics algorithm
  integrate(dt);                 // step forward

  _model->setState(_state);
  _model->resetExternalForces();  // clear external forces
  _model->resetCalculationFlags();
}

/*!
 * Run spring-damper collisions
 * @param dt : timestep
 * @param kp : spring constant
 * @param kd : damping constant
 */
template <typename T>
void DynamicsEngine<T>::updateCollisions(T dt, T kp, T kd) {
  _model->forwardKinematics();
  _contact_constr->UpdateExternalForces(kp, kd, dt);
}

/*!
 * Integrate the floating base state
 * @param dt timestep
 */
template <typename T>
void DynamicsEngine<T>::integrate(T dt) {
  if (_useSpringDamper) {
    Vec3<T> omegaBody = _state.bodyVelocity.template block<3, 1>(0, 0);
    Mat6<T> X = createSXform(quaternionToRotationMatrix(_state.bodyOrientation),
                             _state.bodyPosition);
    RotMat<T> R = rotationFromSXform(X);
    Vec3<T> omega0 = R.transpose() * omegaBody;

    // actual integration
    _state.qd += _dstate.qdd * dt;
    _state.q += _state.qd * dt;

    _state.bodyVelocity += _dstate.dBodyVelocity * dt;
    _state.bodyPosition += _dstate.dBodyPosition * dt;
    _state.bodyOrientation = integrateQuat(_state.bodyOrientation, omega0, dt);
  } else {
    // actual integration
    // Velocity Update by integrating acceleration
    _state.qd += _dstate.qdd * dt;
    _state.bodyVelocity += _dstate.dBodyVelocity * dt;

    // Contact Constraint Velocity Updated
    _contact_constr->UpdateQdot(_state);

    // Prepare body velocity integration
    RotMat<T> R_body = quaternionToRotationMatrix(_state.bodyOrientation);

    _dstate.dBodyPosition =
        R_body.transpose() * _state.bodyVelocity.template block<3, 1>(3, 0);
    Vec3<T> omegaBody = _state.bodyVelocity.template block<3, 1>(0, 0);

    // Position Update
    _state.q += _state.qd * dt;
    _state.bodyPosition += _dstate.dBodyPosition * dt;
    _state.bodyOrientation =
        integrateQuatImplicit(_state.bodyOrientation, omegaBody, dt);
    _dstate.dBodyVelocity = (_state.bodyVelocity - _lastBodyVelocity) / dt;
    _lastBodyVelocity = _state.bodyVelocity;

    bool filter_bodyAcc = true;
    if (filter_bodyAcc){
      // LTI filter
      // https://dsp.stackexchange.com/questions/26248/derive-velocity-and-acceleration-from-position
      T qsi = 0.7;
      T wn = 2 * M_PI * 60;
      T wnT2 = wn*dt * wn*dt;
      T b = 2 * dt * wn * wn;
      T a1 = 4 + (4*qsi*wn*dt) + wnT2;
      T a2 = (2 * wnT2) - 8;
      T a3 = wnT2 - (4*qsi*wn*dt) + 4;

      bodyVel = _state.bodyVelocity;
      bodyAcc = (-a2 * bodyAcc1) - (a3 * bodyAcc2) + b * (bodyVel - bodyVel2);
      bodyAcc *= 1.0 / a1;

      _dstate.dBodyVelocity = bodyAcc;

      bodyAcc1 = bodyAcc;
      bodyAcc2 = bodyAcc1;
      bodyVel1 = bodyVel;
      bodyVel2 = bodyVel1;
    }
  }
}

template class DynamicsEngine<double>;
template class DynamicsEngine<float>;
