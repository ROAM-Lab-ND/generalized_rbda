#pragma once

#include <string>
#include <memory>

#include "Dynamics/Body.h"
#include "Utils/Utilities/SpatialTransforms.h"

namespace grbda
{

    using namespace spatial;

    template <typename Derived>
    struct TreeNode
    {
        const Derived &derived() const { return *static_cast<const Derived *>(this); }
        Derived &derived() { return *static_cast<Derived *>(this); }

        void updateKinematics() { derived().updateKinematics(); }

        const JointCoordinate &jointPosition() const { return joint_state_.position; }
        const JointCoordinate &jointVelocity() const { return joint_state_.velocity; }

        const DVec<double> &vJ() const { return derived().vJ(); }
        const DMat<double> &S() const { return derived().S(); }
        const DMat<double> &S_ring() const { return derived().S_ring(); }

        const SpatialTransform &getAbsoluteTransformForBody(const Body &body) const
        {
            return derived().getAbsoluteTransformForBody(body);
        }
        DVec<double> getVelocityForBody(const Body &body) const
        {
            return derived().getVelocityForBody(body);
        }
        void applyForceToBody(const SVec<double> &force, const Body &body) const
        {
            derived().applyForceToBody(force, body);
        }

        const int position_index_;
        const int num_positions_;
        const int velocity_index_;
        const int num_velocities_;

        const int motion_subspace_dimension_;

        const int index_;
        const std::string name_;
        const int parent_index_;

        JointState joint_state_;

        DVec<double> v_;     // spatial velocity
        DVec<double> c_;     // velocity-product acceleration
        DVec<double> a_;     // spatial acceleration
        DVec<double> f_;     // spatial force across joint
        DVec<double> f_ext_; // net external spatial force acting on the cluster

        DMat<double> I_;  // spatial inertia
        DMat<double> Ic_; // compisite rigid body inertia

        GeneralizedSpatialTransform Xup_;        // spatial transform from parent to child
        GeneralizedAbsoluteSpatialTransform Xa_; // spatial transform from world frame to current frame

    private:
        TreeNode(int index, std::string name, int parent_index, int motion_subspace_dimension,
                 int num_parent_bodies, int position_index, int num_positions,
                 int velocity_index, int num_velocities)
            : position_index_(position_index), num_positions_(num_positions),
              velocity_index_(velocity_index), num_velocities_(num_velocities),
              motion_subspace_dimension_(motion_subspace_dimension),
              index_(index), name_(name), parent_index_(parent_index),
              Xup_(num_parent_bodies)
        {
            I_ = DMat<double>::Zero(motion_subspace_dimension_, motion_subspace_dimension_);
            f_ext_ = DVec<double>::Zero(motion_subspace_dimension_);
        }
        friend Derived;
    };

} // namespace grbda
