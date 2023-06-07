#pragma once

#include <string>
#include <memory>

#include "DynamicsEngine/Body.h"
#include "Utils/Utilities/SpatialTransforms.h"

using namespace spatial;

struct TreeNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TreeNode(int index, std::string name, int parent_index, int motion_subspace_dimension,
             int num_parent_bodies, int position_index, int num_positions,
             int velocity_index, int num_velocities)
        : position_index_(position_index), num_positions_(num_positions),
          velocity_index_(velocity_index), num_velocities_(num_velocities),
          motion_subspace_dimension_(motion_subspace_dimension), index_(index), name_(name),
          parent_index_(parent_index), Xup_(num_parent_bodies)
    {
        q_ = DVec<double>::Zero(num_positions_);
        qd_ = DVec<double>::Zero(num_velocities_);

        I_ = DMat<double>::Zero(motion_subspace_dimension_, motion_subspace_dimension_);
        f_ext_ = DVec<double>::Zero(motion_subspace_dimension_);
    }

    virtual ~TreeNode() {}

    virtual void updateKinematics() = 0;
    virtual const DVec<double> &vJ() const = 0;
    virtual const DMat<double> &S() const = 0;
    virtual const DMat<double> &S_ring() const = 0;

    virtual const SpatialTransform &getAbsoluteTransformForBody(const Body& body) = 0;
    virtual DVec<double> getVelocityForBody(const Body& body) = 0;
    virtual void applyForceToBody(const SVec<double> &force, const Body &body) = 0;

    const int position_index_;
    const int num_positions_;
    const int velocity_index_;
    const int num_velocities_;

    const int motion_subspace_dimension_;

    const int index_;
    const std::string name_;
    const int parent_index_;

    DVec<double> q_;  // joint positions
    DVec<double> qd_; // joint velocities

    DVec<double> v_; // spatial velocity
    DVec<double> c_; // velocity-product acceleration
    DVec<double> a_; // spatial acceleration
    DVec<double> f_; // spatial force across joint
    DVec<double> f_ext_; // net external spatial force acting on the cluster

    DMat<double> I_;  // spatial inertia
    DMat<double> Ic_; // compisite rigid body inertia

    GeneralizedSpatialTransform Xup_;        // spatial transform from parent to child
    GeneralizedAbsoluteSpatialTransform Xa_; // spatial transform from world frame to current frame

};
