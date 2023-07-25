#include "ReflectedInertiaTreeModel.h"

namespace grbda
{

    ReflectedInertiaTreeModel::ReflectedInertiaTreeModel(const ClusterTreeModel &cluster_tree_model,
                                                         bool use_off_diagonal_terms)
        : TreeModel(), use_off_diagonal_terms_(use_off_diagonal_terms)
    {
        gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractIndependentCoordinatesFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    void ReflectedInertiaTreeModel::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        for (const ClusterTreeNode &cluster : cluster_tree_model.clusters())
        {
            const int &nv = cluster.num_velocities_;
            DMat<double> cluster_reflected_inertia = DMat<double>::Zero(nv, nv);
            for (const auto &link_joint_and_ref_inertia :
                 cluster.bodiesJointsAndReflectedInertias())
            {
                const Body &link = std::get<0>(link_joint_and_ref_inertia);
                const auto link_joint = std::get<1>(link_joint_and_ref_inertia);
                const DMat<double> &reflected_inertia = std::get<2>(link_joint_and_ref_inertia);

                const int node_index = (int)nodes_.size();
                const int parent_node_index = getIndexOfParentNodeForBody(link.parent_index_);
                ReflectedInertiaTreeNode node(node_index, link, link_joint, parent_node_index,
                                              position_index_, velocity_index_);
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
        for (const ClusterTreeNode &cluster : cluster_tree_model.clusters())
        {
            const auto joint = cluster.joint_;
            spanning_tree_to_independent_coords_conversion_ =
                appendEigenMatrix(spanning_tree_to_independent_coords_conversion_,
                                  joint->spanningTreeToIndependentCoordsConversion());
        }

        const int num_spanning_coords = spanning_tree_to_independent_coords_conversion_.cols();
        independent_coord_indices_ =
            spanning_tree_to_independent_coords_conversion_.template cast<int>() *
            DVec<int>::LinSpaced(num_spanning_coords, 0, num_spanning_coords - 1);
    }

    void ReflectedInertiaTreeModel::extractContactPointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        contact_name_to_contact_index_ = cluster_tree_model.contact_name_to_contact_index_;
        for (const ContactPoint &contact_point : cluster_tree_model.contactPoints())
        {
            contact_points_.push_back(contact_point);
        }
    }

    void ReflectedInertiaTreeModel::initializeIndependentStates(const DVec<double> &y,
                                                                const DVec<double> &yd)
    {
        for (ReflectedInertiaTreeNode &node : nodes_)
        {
            node.joint_state_.position = y.segment(node.position_index_, node.num_positions_);
            node.joint_state_.velocity = yd.segment(node.velocity_index_, node.num_velocities_);
        }

        initializeExternalForces();
    }

    void ReflectedInertiaTreeModel::resetCache()
    {
        kinematics_updated_ = false;
        mass_matrix_updated_ = false;
        bias_force_updated_ = false;
        articulated_bodies_updated_ = false;
    }

    DMat<double> ReflectedInertiaTreeModel::getMassMatrix()
    {
        compositeRigidBodyAlgorithm();

        if (use_off_diagonal_terms_)
        {
            H_ += reflected_inertia_;
        }
        else
        {
            const DMat<double> reflected_inertia_diag = reflected_inertia_.diagonal().asDiagonal();
            H_ += reflected_inertia_diag;
        }

        return H_;
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
        for (const ReflectedInertiaTreeNode &link_node : nodes_)
            if (link_node.link_.index_ == spanning_tree_index)
                return link_node.link_;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    ReflectedInertiaTreeNode &
    ReflectedInertiaTreeModel::getNodeContainingBody(int spanning_tree_index)
    {
        for (ReflectedInertiaTreeNode &link_node : nodes_)
            if (link_node.link_.index_ == spanning_tree_index)
                return link_node;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    int ReflectedInertiaTreeModel::getIndexOfParentNodeForBody(const int spanning_tree_index)
    {
        if (spanning_tree_index == -1)
            return -1;
        else
            return getNodeContainingBody(spanning_tree_index).index_;
    }

    void ReflectedInertiaTreeModel::contactJacobians()
    {
        forwardKinematics();

        for (ContactPoint &cp : contact_points_)
        {
            const size_t &i = cp.body_index_;
            const ReflectedInertiaTreeNode &node_i = getNodeContainingBody(i);
            const SpatialTransform &Xa = node_i.Xa_[0];
            const Mat3<double> R_link_to_world = Xa.getRotation().transpose();
            Mat6<double> Xout = createSXform(R_link_to_world, cp.local_offset_);

            int j = node_i.index_;
            while (j > -1)
            {
                const ReflectedInertiaTreeNode &node_j = nodes_[j];
                const int &vel_idx = node_j.velocity_index_;
                const int &num_vel = node_j.num_velocities_;

                const D6Mat<double> &S = node_j.S();
                cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

                const Mat6<double> Xup = node_j.Xup_[0].toMatrix();
                Xout = Xout * Xup;

                j = node_j.parent_index_;
            }
        }
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamics(const DVec<double> &tau)
    {
        if (use_off_diagonal_terms_)
            return forwardDynamicsWithOffDiag(tau);
        else
            return forwardDynamicsWithoutOffDiag(tau);
    }

    DVec<double> ReflectedInertiaTreeModel::inverseDynamics(const DVec<double> &ydd)
    {
        return getMassMatrix() * ydd + getBiasForceVector();
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamicsWithOffDiag(const DVec<double> &tau)
    {
        compositeRigidBodyAlgorithm();
        updateBiasForceVector();
        return (H_ + reflected_inertia_).ldlt().solve(tau - C_);
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamicsWithoutOffDiag(const DVec<double> &tau)
    {
        // Forward dynamics via Articulated Body Algorithm
        forwardKinematics();
        updateArticulatedBodies();

        DVec<double> qdd = DVec<double>::Zero(getNumDegreesOfFreedom());

        // Forward Pass - Articulated body bias force
        for (ReflectedInertiaTreeNode &link_node : nodes_)
        {
            link_node.pA_ = generalForceCrossProduct(link_node.v_, DVec<double>(link_node.I_ * link_node.v_));
        }

        // Account for external forces in bias force
        for (int link_node_index : indices_of_nodes_experiencing_external_forces_)
        {
            ReflectedInertiaTreeNode &link_node = nodes_[link_node_index];
            link_node.pA_ -= link_node.Xa_.transformExternalForceVector(link_node.f_ext_);
        }

        // Backward pass - Gauss principal of least constraint
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            ReflectedInertiaTreeNode &link_node = nodes_[i];
            const int &vel_idx = link_node.velocity_index_;
            const int &num_vel = link_node.num_velocities_;
            const auto joint = link_node.joint_;

            link_node.u_ = tau.segment(vel_idx, num_vel) - joint->S().transpose() * link_node.pA_;

            // Articulated body bias force recursion
            if (link_node.parent_index_ >= 0)
            {
                ReflectedInertiaTreeNode &parent_link_node = nodes_[link_node.parent_index_];

                const Mat6<double> Ia = link_node.IA_ -
                                        link_node.U_ * link_node.D_inv_ * link_node.U_.transpose();

                const SVec<double> pa = link_node.pA_ + Ia * link_node.c_ +
                                        link_node.U_ * link_node.D_inv_ * link_node.u_;

                parent_link_node.pA_ += link_node.Xup_.inverseTransformForceVector(pa);
            }
        }

        // Forward Pass - Joint accelerations
        for (ReflectedInertiaTreeNode &link_node : nodes_)
        {
            const int &vel_idx = link_node.velocity_index_;
            const int &num_vel = link_node.num_velocities_;
            const auto joint = link_node.joint_;

            SVec<double> a_temp;
            if (link_node.parent_index_ >= 0)
            {
                const ReflectedInertiaTreeNode &parent_link_node = nodes_[link_node.parent_index_];
                a_temp = link_node.Xup_.transformMotionVector(parent_link_node.a_) + link_node.c_;
            }
            else
            {
                a_temp = link_node.Xup_.transformMotionVector(-gravity_) + link_node.c_;
            }
            qdd.segment(vel_idx, num_vel) =
                link_node.D_inv_ * (link_node.u_ - link_node.U_.transpose() * a_temp);
            link_node.a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }

        return qdd;
    }

    void ReflectedInertiaTreeModel::updateArticulatedBodies()
    {
        if (articulated_bodies_updated_)
            return;

        forwardKinematics();

        // Forward pass
        for (ReflectedInertiaTreeNode &link_node : nodes_)
        {
            link_node.IA_ = link_node.I_;
        }

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            ReflectedInertiaTreeNode &link_node = nodes_[i];
            const auto joint = link_node.joint_;

            const int &vel_idx = link_node.velocity_index_;
            const int &num_vel = link_node.num_velocities_;

            link_node.U_ = link_node.IA_ * joint->S();
            const DMat<double> D = joint->S().transpose() * link_node.U_ +
                                   reflected_inertia_.block(vel_idx, vel_idx, num_vel, num_vel);
            link_node.D_inv_ = D.inverse();

            // Articulated body inertia recursion
            if (link_node.parent_index_ >= 0)
            {
                ReflectedInertiaTreeNode &parent_link_node = nodes_[link_node.parent_index_];
                const DMat<double> Ia = link_node.IA_ -
                                        link_node.U_ * link_node.D_inv_ * link_node.U_.transpose();
                parent_link_node.IA_ += link_node.Xup_.inverseTransformSpatialInertia(Ia);
            }
        }

        articulated_bodies_updated_ = true;
    }

    double ReflectedInertiaTreeModel::applyLocalFrameTestForceAtContactPoint(
        const Vec3<double> &force, const std::string &contact_point_name, DVec<double> &dstate_out)
    {
        forwardKinematics();
        contactJacobians();

        const int contact_point_index = contact_name_to_contact_index_.at(contact_point_name);
        const ContactPoint &contact_point = contact_points_[contact_point_index];

        // ISSUE #23
        const D3Mat<double> J = contact_point.jacobian_.bottomRows<3>();
        const DMat<double> H = getMassMatrix();
        const DMat<double> H_inv = H.inverse();
        const DMat<double> inv_ops_inertia = J * H_inv * J.transpose();
        dstate_out = H_inv * (J.transpose() * force);
        return force.dot(inv_ops_inertia * force);
    }

} // namespace grbda
