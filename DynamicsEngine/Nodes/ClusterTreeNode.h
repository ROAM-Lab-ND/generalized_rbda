#pragma once

#include "TreeNode.h"
#include "DynamicsEngine/GeneralizedJoints/GeneralizedJointTypes.h"

struct ClusterTreeNode : TreeNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ClusterTreeNode(int index, std::string name, std::vector<Body> &bodies,
                    std::shared_ptr<GeneralizedJoints::Base> joint, int parent_index,
                    int num_parent_bodies, int position_index, int velocity_index);

    void updateKinematics() override;
    const DVec<double> &vJ() const override { return joint_->vJ(); }
    const DMat<double> &S() const override { return joint_->S(); }
    const DMat<double> &S_ring() const override { return joint_->S_ring(); }

    const SpatialTransform &getAbsoluteTransformForBody(const Body& body) override;
    DVec<double> getVelocityForBody(const Body& body) override;
    void applyForceToBody(const SVec<double> &force, const Body &body) override;

    bool containsBody(int body_index) const;

    const std::vector<Body> &bodies() const { return bodies_; }
    std::vector<std::pair<Body, JointPtr>> bodiesAndJoints() const;
    std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodiesJointsAndReflectedInertias() const;

    const std::vector<Body> bodies_;
    std::shared_ptr<GeneralizedJoints::Base> joint_;

    // Featherstone quantities
    DMat<double> IA_;    // articulated body inertia
    DVec<double> pA_;    // articulated body bias force
    DMat<double> U_;     // helper variable for ABA
    DMat<double> D_inv_; // helper variable for ABA
    DVec<double> u_;     // helper variable for ABA

    DMat<double> ChiUp_;
    DMat<double> qdd_for_subtree_due_to_subtree_root_joint_qdd;
};
