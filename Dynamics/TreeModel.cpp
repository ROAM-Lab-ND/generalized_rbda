#include "TreeModel.h"

#include "ClusterTreeModel.h"
#include "RigidBodyTreeModel.h"
#include "ReflectedInertiaTreeModel.h"

namespace grbda
{

    template <typename Derived>
    void TreeModel<Derived>::forwardKinematics()
    {
        if (kinematics_updated_)
            return;

            // So I think I need a lambda function for the forward pass, and if I have a std::variant, then I use std::visit, otherwise I just call the function. I think that's the way to do it.

        for (NodeType &node : nodes_)
        {
            node.updateKinematics();

            if (node.parent_index_ >= 0)
            {
                const NodeType& parent_node = nodes_[node.parent_index_];
                node.v_ = node.Xup_.transformMotionVector(parent_node.v_) + node.vJ();
                node.Xa_ = node.Xup_ * parent_node.Xa_;
            }
            else
            {
                node.v_ = node.vJ();
                node.Xa_ = node.Xup_.toAbsolute();
            }


            // ISSUE #9
            node.c_ = node.S_ring() * node.joint_state_.velocity +
                       generalMotionCrossProduct(node.v_, node.vJ());
        }

        // TODO(@MatthewChignoli): Should we do contact kinematics every time we do kinematics?
        contactPointForwardKinematics();

        kinematics_updated_ = true;
    }

    template <typename Derived>
    void TreeModel<Derived>::contactPointForwardKinematics()
    {
        for (auto &cp : contact_points_)
        {
            const Body &body = getBody(cp.body_index_);
            const NodeType &node = getNodeContainingBody(cp.body_index_);

            const SpatialTransform &Xa = node.getAbsoluteTransformForBody(body);
            const SVec<double> v_body = node.getVelocityForBody(body);

            cp.position_ = Xa.inverseTransformPoint(cp.local_offset_);
            cp.velocity_ = spatialToLinearVelocity(Xa.inverseTransformMotionVector(v_body),
                                                   cp.position_);
        }
    }

    template <typename Derived>
    void TreeModel<Derived>::compositeRigidBodyAlgorithm()
    {
        if (mass_matrix_updated_)
            return;

        forwardKinematics();

        // Forward Pass
        for (NodeType &node : nodes_)
            node.Ic_ = node.I_;

        // Backward Pass
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            NodeType &node_i = nodes_[i];
            const int& vel_idx_i = node_i.velocity_index_;
            const int& num_vel_i = node_i.num_velocities_;

            if (node_i.parent_index_ >= 0)
            {
                NodeType& parent_node = nodes_[node_i.parent_index_];
                parent_node.Ic_ += node_i.Xup_.inverseTransformSpatialInertia(node_i.Ic_);
            }

            DMat<double> F = node_i.Ic_ * node_i.S();
            H_.block(vel_idx_i, vel_idx_i, num_vel_i, num_vel_i) = node_i.S().transpose() * F;

            int j = i;
            while (nodes_[j].parent_index_ > -1)
            {
                F = nodes_[j].Xup_.inverseTransformForceSubspace(F);

                j = nodes_[j].parent_index_;
                const int& vel_idx_j = nodes_[j].velocity_index_;
                const int& num_vel_j = nodes_[j].num_velocities_;

                H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j) =
                    F.transpose() * nodes_[j].S();
                H_.block(vel_idx_j, vel_idx_i, num_vel_j, num_vel_i) =
                    H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j).transpose();
            }
        }

        mass_matrix_updated_ = true;
    }

    template <typename Derived>
    void TreeModel<Derived>::updateBiasForceVector()
    {
        if (bias_force_updated_)
            return;

        C_ = recursiveNewtonEulerAlgorithm(DVec<double>::Zero(getNumDegreesOfFreedom()));

        bias_force_updated_ = true;
    }

    template <typename Derived>
    DVec<double> TreeModel<Derived>::recursiveNewtonEulerAlgorithm(const DVec<double> &qdd)
    {
        forwardKinematics();

        DVec<double> tau = DVec<double>::Zero(qdd.rows());

        // Forward Pass
        for (NodeType &node : nodes_)
        {
            const int& vel_idx = node.velocity_index_;
            const int& num_vel = node.num_velocities_;

            if (node.parent_index_ >= 0)
            {
                NodeType& parent_node = nodes_[node.parent_index_];
                node.a_ = node.Xup_.transformMotionVector(parent_node.a_) +
                           node.S() * qdd.segment(vel_idx, num_vel) + node.c_;
            }
            else
            {
                node.a_ = node.Xup_.transformMotionVector(-gravity_) +
                           node.S() * qdd.segment(vel_idx, num_vel) + node.c_;
            }

            node.f_ = node.I_ * node.a_ +
                       generalForceCrossProduct(node.v_, DVec<double>(node.I_ * node.v_));
        }

        // Account for external forces in bias force
        for (int index : indices_of_nodes_experiencing_external_forces_)
        {
            NodeType& node = nodes_[index];
            node.f_ -= node.Xa_.transformExternalForceVector(node.f_ext_);
        }

        // Backward Pass
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            NodeType &node = nodes_[i];
            const int& vel_idx = node.velocity_index_;
            const int& num_vel = node.num_velocities_;

            tau.segment(vel_idx, num_vel) = node.S().transpose() * node.f_;

            if (node.parent_index_ >= 0)
            {
                NodeType &parent_node = nodes_[node.parent_index_];
                parent_node.f_ += node.Xup_.inverseTransformForceVector(node.f_);
            }
        }

        return tau;
    }

    template <typename Derived>
    void TreeModel<Derived>::initializeExternalForces(
        const std::vector<ExternalForceAndBodyIndexPair> &force_and_body_index_pairs)
    {
        // Clear previous external forces
        for (const int index : indices_of_nodes_experiencing_external_forces_)
            nodes_[index].f_ext_.setZero();

        // Apply forces to nodes
        indices_of_nodes_experiencing_external_forces_.clear();
        for (const auto &force_and_body_index : force_and_body_index_pairs)
        {
            const SVec<double> &force = force_and_body_index.force_;
            const int body_index = force_and_body_index.index_;

            const auto &body = getBody(body_index);
            NodeType &node = getNodeContainingBody(body_index);
            node.applyForceToBody(force, body);

            // Add index to vector if vector does not already contain this cluster
            if (!vectorContainsIndex(indices_of_nodes_experiencing_external_forces_, node.index_))
                indices_of_nodes_experiencing_external_forces_.push_back(node.index_);
        }

        resetCache();
    }

    template <typename Derived>
    bool TreeModel<Derived>::vectorContainsIndex(const std::vector<int> vec,
                                                           const int index)
    {
        return std::find(vec.begin(), vec.end(), index) != vec.end();
    }

    template class TreeModel<ClusterTreeModel>;
    template class TreeModel<RigidBodyTreeModel>;
    template class TreeModel<ReflectedInertiaTreeModel>;

} // namespace grbda
