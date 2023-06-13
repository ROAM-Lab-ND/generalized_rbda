#include "ReflectedInertiaTreeModel.h"

namespace grbda
{

    ReflectedInertiaTreeModel::ReflectedInertiaTreeModel(const ClusterTreeModel &cluster_tree_model)
    {
        gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractIndependentCoordinatesFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    void ReflectedInertiaTreeModel::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const int &nv = cluster->num_velocities_;
            DMat<double> cluster_reflected_inertia = DMat<double>::Zero(nv, nv);
            for (const auto &link_joint_and_reflected_inertia :
                 cluster->bodiesJointsAndReflectedInertias())
            {
                const auto &link = std::get<0>(link_joint_and_reflected_inertia);
                const auto link_joint = std::get<1>(link_joint_and_reflected_inertia);
                const auto &reflected_inertia = std::get<2>(link_joint_and_reflected_inertia);

                const int node_index = (int)reflected_inertia_nodes_.size();
                const int parent_node_index = getIndexOfParentNodeForBody(link.parent_index_);
                auto node = std::make_shared<ReflectedInertiaTreeNode>(node_index, link, link_joint,
                                                                       parent_node_index,
                                                                       position_index_,
                                                                       velocity_index_);
                reflected_inertia_nodes_.push_back(node);
                nodes_.push_back(node);

                cluster_reflected_inertia += reflected_inertia;

                position_index_ += link_joint->numPositions();
                velocity_index_ += link_joint->numVelocities();
            }
            reflected_inertia_ = appendEigenMatrix(reflected_inertia_, cluster_reflected_inertia);
        }

        H_ = DMat<double>::Zero(velocity_index_, velocity_index_);
        C_ = DVec<double>::Zero(velocity_index_);
    }

    void ReflectedInertiaTreeModel::extractIndependentCoordinatesFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(0, 0);
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const auto joint = cluster->joint_;
            spanning_tree_to_independent_coords_conversion_ =
                appendEigenMatrix(spanning_tree_to_independent_coords_conversion_,
                                  joint->spanningTreeToIndependentCoordsConversion());
        }

        const int num_bodies = cluster_tree_model.getNumBodies();
        independent_coord_indices_ =
            spanning_tree_to_independent_coords_conversion_.template cast<int>() *
            DVec<int>::LinSpaced(num_bodies, 0, num_bodies - 1);
    }

    void ReflectedInertiaTreeModel::extractContactPointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        contact_name_to_contact_index_ = cluster_tree_model.contact_name_to_contact_index_;
        for (const auto &contact_point : cluster_tree_model.contactPoints())
            contact_points_.push_back(contact_point);
    }

    void ReflectedInertiaTreeModel::initializeIndependentStates(const DVec<double> &y,
                                                                const DVec<double> &yd)
    {
        for (auto &node : reflected_inertia_nodes_)
        {
            node->q_ = y.segment(node->position_index_, node->num_positions_);
            node->qd_ = yd.segment(node->velocity_index_, node->num_velocities_);
        }

        initializeExternalForces();
    }

    void ReflectedInertiaTreeModel::resetCache()
    {
        TreeModel::resetCache();
        articulated_bodies_updated_ = false;
    }

    DMat<double> ReflectedInertiaTreeModel::getMassMatrix()
    {
        compositeRigidBodyAlgorithm();
        return H_ + reflected_inertia_;
    }

    DVec<double> ReflectedInertiaTreeModel::getBiasForceVector()
    {
        updateBiasForceVector();
        return C_;
    }

    // NOTE: The following relationship is true for the rigid body tree model, but not the reflected
    // inertia model: body.index_ = nodes[body.index_].index_
    const Body &ReflectedInertiaTreeModel::getBody(int spanning_tree_index) const
    {
        for (const auto &link_node : reflected_inertia_nodes_)
            if (link_node->link_.index_ == spanning_tree_index)
                return link_node->link_;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    const TreeNodePtr ReflectedInertiaTreeModel::getNodeContainingBody(int spanning_tree_index)
    {
        for (const auto &link_node : reflected_inertia_nodes_)
            if (link_node->link_.index_ == spanning_tree_index)
                return link_node;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    int ReflectedInertiaTreeModel::getIndexOfParentNodeForBody(const int spanning_tree_index)
    {
        if (spanning_tree_index == -1)
            return -1;
        else
            return getNodeContainingBody(spanning_tree_index)->index_;
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamicsHandC(const DVec<double> &tau)
    {
        compositeRigidBodyAlgorithm();
        updateBiasForceVector();
        DVec<double> qdd_ref_inertia = (H_ + reflected_inertia_).inverse() * (tau - C_);
        return qdd_ref_inertia;
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamics(const DVec<double> &tau)
    {
        // Forward dynamics via Articulated Body Algorithm
        forwardKinematics();
        updateArticulatedBodies();

        DVec<double> qdd = DVec<double>::Zero(getNumDegreesOfFreedom());

        // Forward Pass - Articulated body bias force
        for (auto &link_node : reflected_inertia_nodes_)
        {
            link_node->pA_ = generalForceCrossProduct(link_node->v_, DVec<double>(link_node->I_ * link_node->v_));
        }

        // Account for external forces in bias force
        for (int link_node_index : indices_of_nodes_experiencing_external_forces_)
        {
            auto &link_node = reflected_inertia_nodes_[link_node_index];
            link_node->pA_ -= link_node->Xa_.transformExternalForceVector(link_node->f_ext_);
        }

        // Backward pass - Gauss principal of least constraint
        for (int i = (int)reflected_inertia_nodes_.size() - 1; i >= 0; i--)
        {
            auto &link_node = reflected_inertia_nodes_[i];
            const int vel_idx = link_node->velocity_index_;
            const int num_vel = link_node->num_velocities_;
            const auto joint = link_node->joint_;

            link_node->u_ = tau.segment(vel_idx, num_vel) - joint->S().transpose() * link_node->pA_;

            // Articulated body bias force recursion
            if (link_node->parent_index_ >= 0)
            {
                auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];

                Mat6<double> Ia =
                    link_node->IA_ - link_node->U_ * link_node->D_inv_ * link_node->U_.transpose();

                SVec<double> pa =
                    link_node->pA_ + Ia * link_node->c_ + link_node->U_ * link_node->D_inv_ * link_node->u_;

                parent_link_node->pA_ += link_node->Xup_.inverseTransformForceVector(pa);
            }
        }

        // Forward Pass - Joint accelerations
        for (auto &link_node : reflected_inertia_nodes_)
        {
            const int vel_idx = link_node->velocity_index_;
            const int num_vel = link_node->num_velocities_;
            const auto joint = link_node->joint_;

            SVec<double> a_temp;
            if (link_node->parent_index_ >= 0)
            {
                const auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];
                a_temp = link_node->Xup_.transformMotionVector(parent_link_node->a_) + link_node->c_;
            }
            else
            {
                a_temp = link_node->Xup_.transformMotionVector(-gravity_) + link_node->c_;
            }
            qdd.segment(vel_idx, num_vel) =
                link_node->D_inv_ * (link_node->u_ - link_node->U_.transpose() * a_temp);
            link_node->a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }

        return qdd;
    }

    void ReflectedInertiaTreeModel::updateArticulatedBodies()
    {
        if (articulated_bodies_updated_)
            return;

        forwardKinematics();

        // Forward pass
        for (auto &link_node : reflected_inertia_nodes_)
        {
            link_node->IA_ = link_node->I_;
        }

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)reflected_inertia_nodes_.size() - 1; i >= 0; i--)
        {
            auto &link_node = reflected_inertia_nodes_[i];
            const auto joint = link_node->joint_;

            const int vel_idx = link_node->velocity_index_;
            const int num_vel = link_node->num_velocities_;

            link_node->U_ = link_node->IA_ * joint->S();
            const DMat<double> D = joint->S().transpose() * link_node->U_ +
                                   reflected_inertia_.block(vel_idx, vel_idx, num_vel, num_vel);
            link_node->D_inv_ = D.inverse();

            // Articulated body inertia recursion
            if (link_node->parent_index_ >= 0)
            {
                auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];
                DMat<double> Ia =
                    link_node->IA_ - link_node->U_ * link_node->D_inv_ * link_node->U_.transpose();
                parent_link_node->IA_ += link_node->Xup_.inverseTransformSpatialInertia(Ia);
            }
        }

        articulated_bodies_updated_ = true;
    }

} // namespace grbda