#ifndef GBRDA_TREE_NODE_H
#define GBRDA_TREE_NODE_H

#include <string>
#include <memory>

#include "grbda/Dynamics/Body.h"
#include "grbda/Utils/StateRepresentation.h"
#include "grbda/Utils/SpatialTransforms.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{

    template <typename Scalar = double>
    struct TreeNode
    {
        TreeNode(int index, std::string name, int parent_index, int num_parent_bodies, 
                 int motion_subspace_index, int motion_subspace_dimension,
                 int position_index, int num_positions,
                 int velocity_index, int num_velocities)
            : position_index_(position_index), num_positions_(num_positions),
              velocity_index_(velocity_index), num_velocities_(num_velocities),
              motion_subspace_index_(motion_subspace_index),
              motion_subspace_dimension_(motion_subspace_dimension),
              index_(index), name_(name), parent_index_(parent_index),
              Xup_(num_parent_bodies)
        {
            I_ = DMat<Scalar>::Zero(motion_subspace_dimension_, motion_subspace_dimension_);
            f_ext_ = DVec<Scalar>::Zero(motion_subspace_dimension_);
        }

        virtual ~TreeNode() {}

        virtual void updateKinematics() = 0;

        const JointCoordinate<Scalar> &jointPosition() const { return joint_state_.position; }
        const JointCoordinate<Scalar> &jointVelocity() const { return joint_state_.velocity; }
        virtual const DVec<Scalar> &vJ() const = 0;
        virtual const DMat<Scalar> &S() const = 0;
        virtual const DVec<Scalar> &cJ() const = 0;

        virtual const spatial::Transform<Scalar> &getAbsoluteTransformForBody(const Body<Scalar> &body) = 0;
        virtual DVec<Scalar> getVelocityForBody(const Body<Scalar> &body) = 0;
        virtual DVec<Scalar> getAccelerationForBody(const Body<Scalar> &body) = 0;
        virtual void applyForceToBody(const SVec<Scalar> &force, const Body<Scalar> &body) = 0;

        const int position_index_;
        const int num_positions_;
        const int velocity_index_;
        const int num_velocities_;
        const int motion_subspace_index_;
        const int motion_subspace_dimension_;

        const int index_;
        const std::string name_;
        const int parent_index_;

        JointState<Scalar> joint_state_;

        DVec<Scalar> v_;     // spatial velocity
        DVec<Scalar> a_;     // spatial acceleration
        DVec<Scalar> f_;     // spatial force across joint
        DVec<Scalar> f_ext_; // net external spatial force acting on the cluster
        DVec<Scalar> avp_;   // acceleration velocity product

        DMat<Scalar> I_;  // spatial inertia
        DMat<Scalar> Ic_; // compisite rigid body inertia

        spatial::GeneralizedTransform<Scalar> Xup_;        // spatial xform from parent to child
        spatial::GeneralizedAbsoluteTransform<Scalar> Xa_; // spatial xform from world to current

        std::vector<int> supported_end_effectors_;
        std::vector<std::pair<int, int>> nearest_supported_ee_pairs_;
    };

} // namespace grbda

#endif // GBRDA_TREE_NODE_H
