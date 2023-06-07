/*!
 * @file ContactConstraint.cpp
 * @brief ContactConstraint virtual class
 */

#include "Collision/ContactConstraint.h"

/*!
 * Check all points for contact.  If contact occurs, add to list of in-contact points
 * @return Number of points in contact
 */
template <typename T>
size_t ContactConstraint<T>::_CheckContact() {
  _cp_pos_list.clear();
  _cp_local_force_list.clear();
  _cp_penetration_list.clear();

  _cp_frame_list.clear();
  _cp_resti_list.clear();
  _cp_mu_list.clear();
  _idx_list.clear();

  Vec3<T> tmp_vec;
  Mat3<T> cp_frame;

  T penetration;
  for (size_t i(0); i < _model->_nGroundContact; ++i) {
    _cp_force_list[i].setZero();
    for (size_t j(0); j < _nCollision; ++j) {
      if (_collision_list[j]->ContactDetection(_model->_pGC[i], penetration,
                                               cp_frame)) {
        // Contact Happens
        _cp_pos_list.push_back(_model->_pGC[i]);
        _cp_local_force_list.push_back(Vec3<T>::Zero());
        _cp_frame_list.push_back(cp_frame);
        _cp_penetration_list.push_back(penetration);

        _idx_list.push_back(i);

        _cp_resti_list.push_back(_collision_list[j]->getRestitutionCoeff());
        _cp_mu_list.push_back(_collision_list[j]->getFrictionCoeff());
      }
    }
  }
  return _cp_pos_list.size();
}

template <typename T>
int ContactConstraint<T>::_CheckJointLim(FBModelState<T> &state) {
  _joint_idx_list.clear();


  for (int i=0; i < _model->_nJointLim; ++i) {

    if (state.q(_model->_JointLimID[i])<=_model->_JointLimValueLower[i] && state.qd(_model->_JointLimID[i])<=0) {
      // Joint limit Contact Happens
      _joint_idx_list.push_back(_model->_JointLimID[i]);// increments list of joints index for which joint limit is reached
    } else if (state.q(_model->_JointLimID[i])>=_model->_JointLimValueUpper[i] && state.qd(_model->_JointLimID[i])>=0) {
      // Joint limit Contact Happens
      _joint_idx_list.push_back(_model->_JointLimID[i]);// increments list of joints index for which joint limit is reached
    }

  }
  return _joint_idx_list.size();
}


template class ContactConstraint<double>;
template class ContactConstraint<float>;
