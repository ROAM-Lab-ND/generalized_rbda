#ifndef GRBDA_CLUSTER_TREE_NODE_H
#define GRBDA_CLUSTER_TREE_NODE_H

#include "TreeNode.h"
#include "Dynamics/ClusterJoints/ClusterJointTypes.h"

namespace grbda
{

    struct ClusterTreeNode : TreeNode
    {
        typedef std::shared_ptr<ClusterJoints::Base<>> ClusterJointPtr;

        ClusterTreeNode(int index, std::string name, std::vector<Body> &bodies,
                        ClusterJointPtr joint, int parent_index, int num_parent_bodies,
                        int position_index, int velocity_index, int motion_ss_index);

        void updateKinematics() override;
        void updateDinv(const DMat<double> &D);
        const DVec<double> &vJ() const override { return joint_->vJ(); }
        const DMat<double> &S() const override { return joint_->S(); }
        const DVec<double> &cJ() const override { return joint_->cJ(); }

        JointCoordinate<> integratePosition(JointState<> joint_state, double dt)
        {
            return joint_->integratePosition(joint_state, dt);
        }

        const spatial::Transform<> &getAbsoluteTransformForBody(const Body &body) override;
        DVec<double> getVelocityForBody(const Body &body) override;
        void applyForceToBody(const SVec<double> &force, const Body &body) override;

        bool containsBody(int body_index) const;

        const std::vector<Body> &bodies() const { return bodies_; }
        std::vector<std::pair<Body, JointPtr>> bodiesAndJoints() const;
        std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodiesJointsAndReflectedInertias() const;

        const std::vector<Body> bodies_;
        ClusterJointPtr joint_;

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
        DMat<double> K_;
        DMat<double> L_;
    };

} // namespace grbda

#endif // GRBDA_CLUSTER_TREE_NODE_H
