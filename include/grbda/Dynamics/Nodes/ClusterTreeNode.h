#ifndef GRBDA_CLUSTER_TREE_NODE_H
#define GRBDA_CLUSTER_TREE_NODE_H

#include "grbda/Dynamics/Nodes/TreeNode.h"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

namespace grbda
{

    template <typename Scalar = double>
    struct ClusterTreeNode : TreeNode<Scalar>
    {
        typedef typename CorrectMatrixInverseType<Scalar>::type InverseType;
        typedef std::shared_ptr<ClusterJoints::Base<Scalar>> ClusterJointPtr;
        typedef std::pair<Body<Scalar>, JointPtr<Scalar>> BodyJointPair;
        typedef std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>> BodyJointRefInertiaTriple;

        ClusterTreeNode(int index, std::string name, std::vector<Body<Scalar>> &bodies,
                        ClusterJointPtr joint, int parent_index, int num_parent_bodies,
                        int position_index, int velocity_index, int motion_ss_index);

        void updateKinematics() override;
        void updateDinv(const DMat<Scalar> &D);
        const DVec<Scalar> &vJ() const override { return joint_->vJ(); }
        const DMat<Scalar> &S() const override { return joint_->S(); }
        const DVec<Scalar> &cJ() const override { return joint_->cJ(); }

        const spatial::Transform<Scalar> &getAbsoluteTransformForBody(const Body<Scalar> &body) override;
        DVec<Scalar> getVelocityForBody(const Body<Scalar> &body) override;
        DVec<Scalar> getAccelerationForBody(const Body<Scalar> &body) override;
        void applyForceToBody(const SVec<Scalar> &force, const Body<Scalar> &body) override;

        bool containsBody(int body_index) const;

        const std::vector<Body<Scalar>> &bodies() const { return bodies_; }
        std::vector<BodyJointPair> bodiesAndJoints() const;
        std::vector<BodyJointRefInertiaTriple> bodiesJointsAndReflectedInertias() const;

        const std::vector<Body<Scalar>> bodies_;
        ClusterJointPtr joint_;

        // Featherstone quantities
        DMat<Scalar> IA_;       // articulated body inertia
        DVec<Scalar> pA_;       // articulated body bias force
        DMat<Scalar> U_;        // helper variable for ABA
        InverseType D_inv_;     // helper variable for ABA
        DVec<Scalar> u_;        // helper variable for ABA
        DMat<Scalar> D_inv_UT_; // D_inv_ * U_.transpose();
        DVec<Scalar> D_inv_u_;  // D_inv_ * u_;
        DMat<Scalar> Ia_;

        DMat<Scalar> ChiUp_;
        DMat<Scalar> qdd_for_subtree_due_to_subtree_root_joint_qdd;
        DMat<Scalar> K_;
        DMat<Scalar> L_;
    };

} // namespace grbda

#endif // GRBDA_CLUSTER_TREE_NODE_H
