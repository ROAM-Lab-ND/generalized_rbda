#include "RigidBodyTreeModel.h"

namespace grbda
{

    RigidBodyTreeModel::RigidBodyTreeModel(const ClusterTreeModel &cluster_tree_model,
                                           const ForwardDynamicsMethod fd_method)
    {
        forward_dynamics_method_ = fd_method;
        gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractLoopClosureFunctionsFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    void RigidBodyTreeModel::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            for (const auto &body_and_joint : cluster->bodiesAndJoints())
            {
                const auto &body = body_and_joint.first;
                const auto &joint = body_and_joint.second;
                auto node = std::make_shared<RigidBodyTreeNode>(body, joint, position_index_,
                                                                velocity_index_);
                rigid_body_nodes_.push_back(node);
                nodes_.push_back(node);
                position_index_ += joint->numPositions();
                velocity_index_ += joint->numVelocities();
            }
        }
        H_ = DMat<double>::Zero(velocity_index_, velocity_index_);
        C_ = DVec<double>::Zero(velocity_index_);
    }

    void RigidBodyTreeModel::extractLoopClosureFunctionsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        gamma_ = [cluster_tree_model](DVec<double> q)
        {
            DVec<double> q_full = DVec<double>::Zero(0);
            for (const auto &cluster : cluster_tree_model.clusters())
            {
                const int pos_idx = cluster->position_index_;
                const int num_pos = cluster->num_positions_;
                const auto joint = cluster->joint_;
                q_full = appendEigenVector(q_full, joint->gamma(q.segment(pos_idx, num_pos)));
            }
            return q_full;
        };

        G_ = DMat<double>::Zero(0, 0);
        g_ = DVec<double>::Zero(0);
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const auto joint = cluster->joint_;
            G_ = appendEigenMatrix(G_, joint->G());
            g_ = appendEigenVector(g_, joint->g());
        }
        G_pinv_ = G_.completeOrthogonalDecomposition().pseudoInverse();
        G_tranpose_pinv_ = G_.transpose().completeOrthogonalDecomposition().pseudoInverse();

        K_ = DMat<double>::Zero(0, 0);
        k_ = DVec<double>::Zero(0);
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const auto joint = cluster->joint_;
            K_ = appendEigenMatrix(K_, joint->K());
            k_ = appendEigenVector(k_, joint->k());
        }
    }

    void RigidBodyTreeModel::extractContactPointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        contact_name_to_contact_index_ = cluster_tree_model.contact_name_to_contact_index_;
        for (const auto &contact_point : cluster_tree_model.contactPoints())
            contact_points_.push_back(contact_point);
    }

    void RigidBodyTreeModel::initializeStates(const DVec<double> q, const DVec<double> &qd)
    {
        for (auto &node : rigid_body_nodes_)
        {
            node->q_ = q.segment(node->position_index_, node->num_positions_);
            node->qd_ = qd.segment(node->velocity_index_, node->num_velocities_);
        }

        initializeExternalForces();
    }

    void RigidBodyTreeModel::initializeIndependentStates(const DVec<double> &y, const DVec<double> &yd)
    {
        DVec<double> q = gamma_(y);
        DVec<double> qd = G_ * yd;

        for (auto &node : rigid_body_nodes_)
        {
            node->q_ = q.segment(node->position_index_, node->num_positions_);
            node->qd_ = qd.segment(node->velocity_index_, node->num_velocities_);
        }

        initializeExternalForces();
    }

    DMat<double> RigidBodyTreeModel::getMassMatrix()
    {
        compositeRigidBodyAlgorithm();
        return G_.transpose() * H_ * G_;
    }

    DVec<double> RigidBodyTreeModel::getBiasForceVector()
    {
        updateBiasForceVector();
        return G_.transpose() * (C_ + H_ * g_);
    }

} // namespace grbda
