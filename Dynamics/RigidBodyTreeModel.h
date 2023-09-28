#ifndef GRBDA_DYNAMICS_RIGID_BODY_TREE_MODEL_H
#define GRBDA_DYNAMICS_RIGID_BODY_TREE_MODEL_H

#include "ClusterTreeModel.h"
#include "Nodes/RigidBodyTreeNode.h"
#include "Utils/Factorization.h"

namespace grbda
{

    enum class FwdDynMethod
    {
        Projection,
        LagrangeMultiplierCustom,
        LagrangeMultiplierEigen
    };

    template <typename Scalar>
    using RigidBodyTreeNodePtr = std::shared_ptr<RigidBodyTreeNode<Scalar>>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    template <typename Scalar = double>
    class RigidBodyTreeModel : public TreeModel<Scalar>
    {
    public:
        RigidBodyTreeModel(const ClusterTreeModel<Scalar> &cluster_tree_model,
                           const FwdDynMethod fd_method = FwdDynMethod::Projection);
        ~RigidBodyTreeModel() {}

        void setForwardDynamicsMethod(FwdDynMethod fd_method)
        {
            forward_dynamics_method_ = fd_method;
        }

        int getNumBodies() const override { return (int)rigid_body_nodes_.size(); }

        const Body<Scalar> &getBody(int index) const override
        {
            return rigid_body_nodes_[index]->body_;
        }
        const TreeNodePtr<Scalar> getNodeContainingBody(int index) override
        {
            return rigid_body_nodes_[index];
        }

        void setState(const DVec<Scalar> &q, const DVec<Scalar> &qd);

        void updateLoopConstraints();

        Vec3<Scalar> getPosition(const std::string &body_name) override;
        Mat3<Scalar> getOrientation(const std::string &body_name) override;
        Vec3<Scalar> getLinearVelocity(const std::string &body_name) override;
        Vec3<Scalar> getAngularVelocity(const std::string &body_name) override;

        D6Mat<Scalar> contactJacobianBodyFrame(const std::string &cp_name) override;
        const D6Mat<Scalar> &contactJacobianWorldFrame(const std::string &cp_name) override;

        DVec<Scalar> forwardDynamics(const DVec<Scalar> &tau) override;
        DVec<Scalar> inverseDynamics(const DVec<Scalar> &ydd) override;
        DMat<Scalar> inverseOperationalSpaceInertiaMatrix() override;

        Scalar applyTestForce(const std::string &contact_point_name,
                              const Vec3<Scalar> &force, DVec<Scalar> &dstate_out) override;

        DMat<Scalar> getMassMatrix() override;
        DVec<Scalar> getBiasForceVector() override;

        DVec<Scalar> qddToYdd(DVec<Scalar> qdd) const
        {
            return loop_constraints_.G_pinv() * (qdd - loop_constraints_.g());
        }

        DVec<Scalar> yddToQdd(DVec<Scalar> ydd) const
        {
            return loop_constraints_.G() * ydd + loop_constraints_.g();
        }

    private:
        void extractRigidBodiesAndJointsFromClusterModel(
            const ClusterTreeModel<Scalar> &cluster_tree_model);
        void extractLoopClosureFunctionsFromClusterModel(
            const ClusterTreeModel<Scalar> &cluster_tree_model);
        void extractContactPointsFromClusterModel(
            const ClusterTreeModel<Scalar> &cluster_tree_model);
        void extractExpandedTreeConnectivity();

        void resetCache() override
        {
            TreeModel<Scalar>::resetCache();
            loop_constraints_updated_ = false;
        }

        FwdDynMethod forward_dynamics_method_;

        LoopConstraint::Collection<Scalar> loop_constraints_;
        bool loop_constraints_updated_ = false;

        std::vector<RigidBodyTreeNodePtr<Scalar>> rigid_body_nodes_;
        std::unordered_map<std::string, int> body_name_to_body_index_;

        // NOTE: The expanded tree parent indices represent the parent indices for the connectivty 
        // graph resulting from treating multi-dof joints as multiple single-dof joints.
        std::vector<int> expanded_tree_parent_indices_;

        DVec<Scalar> q_;
        DVec<Scalar> qd_;

    };

} // namespace grbda

#endif // GRBDA_DYNAMICS_RIGID_BODY_TREE_MODEL_H
