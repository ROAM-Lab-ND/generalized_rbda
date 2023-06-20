#pragma once

#include "ClusterTreeModel.h"
#include "Factorization.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    enum class ForwardDynamicsMethod
    {
        Projection,
        LagrangeMultiplier
    };

    using RigidBodyTreeNodePtr = std::shared_ptr<RigidBodyTreeNode>;

    /*!
     * Class to represent a floating base rigid body model with rotors and ground
     * contacts. No concept of state.
     */
    class RigidBodyTreeModel : public TreeModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RigidBodyTreeModel(const ClusterTreeModel &cluster_tree_model,
                           const ForwardDynamicsMethod fd_method = ForwardDynamicsMethod::Projection);
        ~RigidBodyTreeModel() {}

        void setForwardDynamicsMethod(ForwardDynamicsMethod fd_method)
        {
            forward_dynamics_method_ = fd_method;
        }

        int getNumBodies() const override { return (int)rigid_body_nodes_.size(); }

        const Body &getBody(int index) const override { return rigid_body_nodes_[index]->body_; }
        const TreeNodePtr getNodeContainingBody(int index) override { return rigid_body_nodes_[index]; }

        // TODO(@MatthewChignoli): Cleanup
        void initializeStates(const DVec<double> &q, const DVec<double> &qd);
        void initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd) override;

        void initializeState(const ModelState &model_state) override
        {
            // Convert model_state to a vector of spanning q and qd
            DVec<double> joint_pos = DVec<double>::Zero(0);
            DVec<double> joint_vel = DVec<double>::Zero(0);
            for (const auto &joint_state : model_state)
            {
                // TODO(@MatthewChignoli): We need to convert to all spanning...
                joint_pos = appendEigenVector(joint_pos, joint_state.position);
                joint_vel = appendEigenVector(joint_vel, joint_state.velocity);
            }

            // TODO(@MatthewChignoli): Delete the initializeStates function? And just put the code here?
            initializeStates(joint_pos, joint_vel);
        }

        DVec<double> forwardDynamics(const DVec<double> &tau) override;

        DMat<double> getMassMatrix() override;
        DVec<double> getBiasForceVector() override;

        DVec<double> qddToYdd(DVec<double> qdd) const { return G_pinv_ * (qdd - g_); }
        DVec<double> yddToQdd(DVec<double> ydd) const { return G_ * ydd + g_; }
        void extractLoopClosureFunctionsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

    private:
        void extractRigidBodiesAndJointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);
        void extractContactPointsFromClusterModel(const ClusterTreeModel &cluster_tree_model);

        ForwardDynamicsMethod forward_dynamics_method_;

        DVecFcn<double> gamma_;
        DMat<double> G_;
        DMat<double> G_pinv_;
        DMat<double> G_tranpose_pinv_;
        DVec<double> g_;

        DVecFcn<double> phi_;
        DMat<double> K_;
        DVec<double> k_;

        std::vector<RigidBodyTreeNodePtr> rigid_body_nodes_;
    };

} // namespace grbda
