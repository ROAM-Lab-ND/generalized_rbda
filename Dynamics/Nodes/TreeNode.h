#ifndef GBRDA_TREE_NODE_H
#define GBRDA_TREE_NODE_H

#include <string>
#include <memory>

#include "Dynamics/Body.h"
#include "Utils/StateRepresentation.h"
#include "Utils/SpatialTransforms.h"

namespace grbda
{

    using namespace spatial;

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
            I_ = DMat<double>::Zero(motion_subspace_dimension_, motion_subspace_dimension_);
            f_ext_ = DVec<double>::Zero(motion_subspace_dimension_);
        }

        virtual ~TreeNode() {}

        virtual void updateKinematics() = 0;

        const JointCoordinate &jointPosition() const { return joint_state_.position; }
        const JointCoordinate &jointVelocity() const { return joint_state_.velocity; }
        virtual const DVec<double> &vJ() const = 0;
        virtual const DMat<double> &S() const = 0;
        virtual const DVec<double> &cJ() const = 0;

        virtual const SpatialTransform &getAbsoluteTransformForBody(const Body &body) = 0;
        virtual DVec<double> getVelocityForBody(const Body &body) = 0;
        virtual void applyForceToBody(const SVec<double> &force, const Body &body) = 0;

        const int position_index_;
        const int num_positions_;
        const int velocity_index_;
        const int num_velocities_;
        const int motion_subspace_index_;
        const int motion_subspace_dimension_;

        const int index_;
        const std::string name_;
        const int parent_index_;

        JointState joint_state_;

        DVec<double> v_;     // spatial velocity
        DVec<double> a_;     // spatial acceleration
        DVec<double> f_;     // spatial force across joint
        DVec<double> f_ext_; // net external spatial force acting on the cluster
        DVec<double> avp_;   // acceleration velocity product

        DMat<double> I_;  // spatial inertia
        DMat<double> Ic_; // compisite rigid body inertia

        GeneralizedSpatialTransform Xup_;        // spatial xform from parent to child
        GeneralizedAbsoluteSpatialTransform Xa_; // spatial xform from world frame to current frame

        std::vector<int> supported_end_effectors_;
        std::vector<std::pair<int, int>> nearest_supported_ee_pairs_;
    };

} // namespace grbda

#endif // GBRDA_TREE_NODE_H
