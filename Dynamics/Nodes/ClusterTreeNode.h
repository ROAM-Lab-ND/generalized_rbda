#pragma once

#include "TreeNode.h"
#include "Dynamics/GeneralizedJoints/GeneralizedJointTypes.h"

namespace grbda
{

    // TODO(@MatthewChignoli): We will need to provide the generalized joint type as a template to this class

    template <typename GenJointType = GeneralizedJoints::Base>
    struct ClusterTreeNode : TreeNode<ClusterTreeNode<>>
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ClusterTreeNode(int index, std::string name, std::vector<Body> &bodies,
                        std::shared_ptr<GenJointType> joint, int parent_index,
                        int num_parent_bodies, int position_index, int velocity_index);

        void updateKinematics();
        void updateDinv(const DMat<double> &D);
        const DVec<double> &vJ() const { return joint_->vJ(); }
        const DMat<double> &S() const { return joint_->S(); }
        const DMat<double> &S_ring() const { return joint_->S_ring(); }

        // ISSUE #14
        // TODO(@MatthewChignoli): Should this actually be a virtual function in TreeNode.h?
        JointCoordinate integratePosition(JointState joint_state, double dt) const
        {
            if (joint_state.position.isSpanning() && joint_state.velocity.isSpanning())
            {
                joint_state.position += joint_state.velocity * dt;
            }
            else if (joint_state.position.isSpanning())
            {
                joint_state.position += joint_->G() * joint_state.velocity * dt;
            }
            else if (joint_state.velocity.isSpanning())
            {
                throw std::runtime_error("Velocity is spanning but position is not. This is not supported.");
            }
            else
            {
                joint_state.position += joint_state.velocity * dt;
            }

            return joint_state.position;
        }

        const SpatialTransform &getAbsoluteTransformForBody(const Body &body) const;
        DVec<double> getVelocityForBody(const Body &body) const;
        void applyForceToBody(const SVec<double> &force, const Body &body);

        bool containsBody(int body_index) const;

        const std::vector<Body> &bodies() const { return bodies_; }
        std::vector<std::pair<Body, JointPtr>> bodiesAndJoints() const;
        std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodiesJointsAndReflectedInertias() const;

        const std::vector<Body> bodies_;
        std::shared_ptr<GenJointType> joint_;

        // Featherstone quantities
        DMat<double> IA_;                                // articulated body inertia
        DVec<double> pA_;                                // articulated body bias force
        DMat<double> U_;                                 // helper variable for ABA
        Eigen::ColPivHouseholderQR<DMat<double>> D_inv_; // helper variable for ABA
        DVec<double> u_;                                 // helper variable for ABA
        DMat<double> D_inv_UT_;                          // D_inv_ * U_.transpose();
        DVec<double> D_inv_u_;                           // D_inv_ * u_;
        DMat<double> Ia_;

        DMat<double> ChiUp_;
        DMat<double> qdd_for_subtree_due_to_subtree_root_joint_qdd;
    };

} // namespace grbda
