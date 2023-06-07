/*!
 * @file ContactImpulse.cpp
 * @brief Implementation of impulse-based contact dynamics
 *
 * These are the default contact dynamics of the simulator.
 */

#include "ContactImpulse.h"
#include <utilities/pretty_print.h>

#include <humanoid/robots/Humanoid.h>
#include <iostream>
/*!
 * Update the q_dot values of a model based on impulse based contact dynamics
 * @param state : states to update
 */
template <typename T>
void ContactImpulse<T>::UpdateQdot(FBModelState<T>& state) {
  this->_nContact = this->_CheckContact();

  this->_nJointContact = this->_CheckJointLim(state); //returns nb of joint contacts occuring

  // if (this->_nJointContact>0){
  //     std::cout << this->_nJointContact <<"/"<< this->_model->_nJointLim <<'\n';
  // }

  if (this->_nContact > 0||this->_nJointContact > 0) {
    DVec<T> qdot(_nDof);

    for (size_t i(0); i < 6; ++i) {
      qdot[i] = state.bodyVelocity[i];
    }
    for (size_t i(0); i < _nDof - 6; ++i) {
      qdot[i + 6] = state.qd[i];
    }

    _UpdateVelocity(qdot,state);

    for (size_t i(0); i < 6; ++i) {
      state.bodyVelocity[i] = qdot[i];
    }
    for (size_t i(0); i < _nDof - 6; ++i) {
      state.qd[i] = qdot[i + 6];
    }

    // Update contact force w.r.t. global frame
    // This is not neccessary for dynamics, but
    // the global contact forces are used for
    // graphics
    for (size_t i(0); i < this->_nContact; ++i) {
      // Divide by dt to convert impulses to forces
      this->_cp_force_list[this->_idx_list[i]] =
          this->_cp_frame_list[i] * this->_cp_local_force_list[i] / _dt;
    }
  }
}

template <typename T>
void ContactImpulse<T>::_UpdateVelocity(DVec<T>& qdot,FBModelState<T> &state) {
  // lambda is the operational-space matrix
  T* lambda_list_x = new T[this->_nContact];
  T* lambda_list_y = new T[this->_nContact];
  T* lambda_list_z = new T[this->_nContact];

  T* des_vel_list_z = new T[this->_nContact];
  T* des_vel_list_tan = new T[this->_nContact];

  T* min_list_z = new T[this->_nContact];
  T* max_list_z = new T[this->_nContact];

  T* min_list_tan = new T[this->_nContact];
  T* max_list_tan = new T[this->_nContact];


  T* H_dir_list = new T[this->_nJointContact]; // list of directional joint space inertia matrix

  vectorAligned<DMat<T> > J_dir_joint_list(this->_nJointContact);

  vectorAligned<DVec<T> > AinvB_list_x(this->_nContact);
  vectorAligned<DVec<T> > AinvB_list_y(this->_nContact);
  vectorAligned<DVec<T> > AinvB_list_z(this->_nContact);

  vectorAligned<D3Mat<T> > Jc_list(this->_nContact);
  // For check
  vectorAligned<Vec3<T> > cp_vel_list(this->_nContact);

  DVec<T> AinvB;
  Vec3<T> cp_local_vel;
  Vec3<T> direction;
  this->_model->contactJacobians();

  // Prepare Matrix and Vector
  for (size_t i(0); i < this->_nContact; ++i) {
    // Lambda & Ainv*J'
    lambda_list_x[i] = this->_model->applyTestForce(
                this->_idx_list[i], 
                this->_cp_frame_list[i].template block<3, 1>(0, 0),   // x-direction of the local frame
                AinvB_list_x[i]);
    lambda_list_x[i] = 1. / lambda_list_x[i];

    lambda_list_y[i] = this->_model->applyTestForce(
                this->_idx_list[i], 
                this->_cp_frame_list[i].template block<3, 1>(0, 1),   // y-direction of the local frame
                AinvB_list_y[i]);
    lambda_list_y[i] = 1. / lambda_list_y[i];

    lambda_list_z[i] = this->_model->applyTestForce(
                this->_idx_list[i], 
                this->_cp_frame_list[i].template block<3, 1>(0, 2),   // z-direction of the local frame
                AinvB_list_z[i]);
    lambda_list_z[i] = 1. / lambda_list_z[i];




    // Local Velocity
    cp_local_vel =
        this->_cp_frame_list[i].transpose() * this->_model->_vGC[this->_idx_list[i]];

    // Vec3<T> vGc = this->_model->_vGC[this->_idx_list[i]];
    // std::cout << "vGC: " << vGc[0] << ", " << vGc[1] << ", " << vGc[2] << std::endl;

    // Desired Velocity
    des_vel_list_tan[i] = 0.;
    if (cp_local_vel[2] < 0.) {
      des_vel_list_z[i] =
          -this->_cp_resti_list[i] * cp_local_vel[2] -
          _penetration_recover_ratio * this->_cp_penetration_list[i];
    } else {
      des_vel_list_z[i] =
          std::max(cp_local_vel[2],
                   -_penetration_recover_ratio * this->_cp_penetration_list[i]);
    }

    // Contact Jacobian
    Jc_list[i] =
        this->_cp_frame_list[i].transpose() * this->_model->_Jc[this->_idx_list[i]];

    min_list_z[i] = 0.;
    max_list_z[i] = 1.e5;
  }

  DMat<T> H_inv;
  if (this->_nJointContact>0){
    H_inv = this->_model->massMatrix().inverse();
  }

  for (int i=0; i < this->_nJointContact; ++i){// loop over all detected joint impacts
    DMat<T> Jtemp = DMat<T>::Zero(1, _nDof);
    Jtemp(this->_joint_idx_list[i]+6) = 1;
    J_dir_joint_list[i] = Jtemp;
    DMat<T> H_dir_temp = (J_dir_joint_list[i]*H_inv*J_dir_joint_list[i].transpose()).inverse();
    H_dir_list[i] = H_dir_temp(0);//just take 0th element since its a scaler anyway
  }

  // Update Velocity & Find Impulse Force
  for (size_t iter(0); iter < _iter_lim; ++iter) {
    _iter_sum = 0;

    // Joint Space Impacts ******************************************************
    for (int jt_idx = 0;jt_idx < this->_nJointContact; ++jt_idx) {
      qdot = qdot + H_inv*J_dir_joint_list[jt_idx].transpose()*H_dir_list[jt_idx]*(-J_dir_joint_list[jt_idx]*qdot);
      //pretty_print(qdot(this->_joint_idx_list[jt_idx]+6),std::cout,"qd");
    }

    // Normal (Z) *********************************************************
    _UpdateQdotOneDirection(2, Jc_list, lambda_list_z, AinvB_list_z,
                            des_vel_list_z, min_list_z, max_list_z, qdot);

    // X ******************************************************************
    for (size_t i(0); i < this->_nContact; ++i) {
      max_list_tan[i] = this->_cp_mu_list[i] * this->_cp_local_force_list[i][2];
      min_list_tan[i] = -max_list_tan[i];
    }
    _UpdateQdotOneDirection(0, Jc_list, lambda_list_x, AinvB_list_x,
                            des_vel_list_tan, min_list_tan, max_list_tan, qdot);

    // Y ******************************************************************
    _UpdateQdotOneDirection(1, Jc_list, lambda_list_y, AinvB_list_y,
                            des_vel_list_tan, min_list_tan, max_list_tan, qdot);

    if (_iter_sum < 1) {
      // printf("converged: %lu \n", _iter_sum);
      break;
    }
    _iter_sum = 0;
  }


  delete[] lambda_list_x;
  delete[] lambda_list_y;
  delete[] lambda_list_z;

  delete[] des_vel_list_z;
  delete[] des_vel_list_tan;

  delete[] min_list_z;
  delete[] max_list_z;

  delete[] min_list_tan;
  delete[] max_list_tan;

  delete[] H_dir_list;
}


template <typename T>
void ContactImpulse<T>::_UpdateQdotOneDirection(
                        size_t idx, 
                        const vectorAligned<D3Mat<T> >& Jc_list, 
                        const T* lambda_list,
                        const vectorAligned<DVec<T> > AinvB_list, 
                        const T* des_vel_list,
                        const T* min_list, 
                        const T* max_list, 
                        DVec<T>& qdot) {
  T dforce, pre_force, cp_vel, dforce_sum;
  T dvel, dvel_sum;

  for (size_t iter(0); iter < _iter_lim; ++iter) {
    dforce_sum = 0;
    dvel_sum = 0;
    for (size_t i(0); i < this->_nContact; ++i) {
      cp_vel = (Jc_list[i].block(idx, 0, 1, _nDof) * qdot)(0, 0);

      dforce = (des_vel_list[i] - cp_vel) * lambda_list[i];
      dvel = des_vel_list[i] - cp_vel;

      pre_force = this->_cp_local_force_list[i][idx];
      this->_cp_local_force_list[i][idx] =
          std::max(min_list[i], std::min(pre_force + dforce, max_list[i]));

      dforce = this->_cp_local_force_list[i][idx] - pre_force;

      qdot += (AinvB_list[i] * dforce);
      dforce_sum += fabs(dforce);

      dvel_sum += fabs(dvel);
    }
    // if (dforce_sum < _tol) {
    //   _iter_sum += iter;
    //   // printf("dforce sum is small enough: %f\n", dforce_sum);
    //   // printf("iter sum is: %i\n", (int)_iter_sum);
    //   break;
    // }
    if (dvel_sum < _tol){
      _iter_sum += iter;
      break;
    }
  }
}

template class ContactImpulse<double>;
template class ContactImpulse<float>;
