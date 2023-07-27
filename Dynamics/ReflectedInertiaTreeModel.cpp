#include "ReflectedInertiaTreeModel.h"

namespace grbda
{

    using namespace ReflectedInertiaNodeVisitors;

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
        for (const ClusterTreeModel::NodeTypeVariants &cluster : cluster_tree_model.clusterVariants())
        {
            const int nv = numVelocities(cluster);
            DMat<double> cluster_reflected_inertia = DMat<double>::Zero(nv, nv);
            for (const auto &link_joint_and_ref_inertia : ClusterNodeVisitors::bodiesJointsAndReflectedInertias(cluster))
            {
                const Body &link = std::get<0>(link_joint_and_ref_inertia);
                const auto link_joint = std::get<1>(link_joint_and_ref_inertia);
                const DMat<double> &reflected_inertia = std::get<2>(link_joint_and_ref_inertia);

                const int node_index = (int)nodes_variants_.size();
                const int parent_node_index = getIndexOfParentNodeForBody(link.parent_index_);
                ReflectedInertiaTreeNode node(node_index, link, link_joint, parent_node_index,
                                              position_index_, velocity_index_);
                nodes_variants_.push_back(node);

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
        for (const ClusterTreeModel::NodeTypeVariants &cluster : cluster_tree_model.clusterVariants())
        {
            const auto joint = ClusterNodeVisitors::getJoint(cluster);
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

        for (NodeTypeVariants &node : nodes_variants_)
        {
            JointCoordinate position(y.segment(positionIndex(node), numPositions(node)), false);
            JointCoordinate velocity(yd.segment(velocityIndex(node), numVelocities(node)), false);
            setJointState(node, JointState(position, velocity));
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
        for (const NodeTypeVariants &link_node : nodes_variants_)
            if (getLink(link_node).index_ == spanning_tree_index)
                return getLink(link_node);
        throw std::runtime_error("1That body does not exist in the link and rotor tree model");
    }

    ReflectedInertiaTreeModel::NodeTypeVariants &
    ReflectedInertiaTreeModel::getNodeVariantContainingBody(int spanning_tree_index)
    {
        for (NodeTypeVariants &link_node : nodes_variants_)
            if (getLink(link_node).index_ == spanning_tree_index)
                return link_node;
        throw std::runtime_error("3That body does not exist in the link and rotor tree model");
    }

    int ReflectedInertiaTreeModel::getIndexOfParentNodeForBody(const int spanning_tree_index)
    {
        if (spanning_tree_index == -1)
            return -1;
        else
            return index(getNodeVariantContainingBody(spanning_tree_index));
    }

    void ReflectedInertiaTreeModel::contactJacobians()
    {
        forwardKinematics();

        for (ContactPoint &cp : contact_points_)
        {
            const size_t &i = cp.body_index_;
            const NodeTypeVariants &node_i = getNodeVariantContainingBody(i);
            const SpatialTransform &X = Xa(node_i)[0];
            const Mat3<double> R_link_to_world = X.getRotation().transpose();
            Mat6<double> Xout = createSXform(R_link_to_world, cp.local_offset_);

            int j = index(node_i);
            while (j > -1)
            {
                const NodeTypeVariants &node_j = nodes_variants_[j];
                const int vel_idx = velocityIndex(node_j);
                const int num_vel = numVelocities(node_j);

                const D6Mat<double> &S = motionSubspace(node_j);
                cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

                const Mat6<double> Xup_j = Xup(node_j)[0].toMatrix();
                Xout = Xout * Xup_j;

                j = parentIndex(node_j);
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
        for (NodeTypeVariants &link_node : nodes_variants_)
        {
            updateArticulatedBiasForce(link_node);
            // link_node.pA_ = generalForceCrossProduct(link_node.v_, DVec<double>(link_node.I_ * link_node.v_));
        }

        // Account for external forces in bias force
        for (int link_node_index : indices_of_nodes_experiencing_external_forces_)
        {
            NodeTypeVariants& link_node = nodes_variants_[link_node_index];
            pA(link_node) -= Xa(link_node).transformExternalForceVector(externalForce(link_node));
        }

        // Backward pass - Gauss principal of least constraint
        for (int i = (int)nodes_variants_.size() - 1; i >= 0; i--)
        {
            NodeTypeVariants &link_node = nodes_variants_[i];
            const int vel_idx = velocityIndex(link_node);
            const int num_vel = numVelocities(link_node);
            const int parent_idx = parentIndex(link_node); 
            const auto joint = getJoint(link_node);

            // link_node.u_ = tau.segment(vel_idx, num_vel) - joint->S().transpose() * link_node.pA_;
            u(link_node) = tau.segment(vel_idx, num_vel) - joint->S().transpose() * pA(link_node);

            // Articulated body bias force recursion
            if (parent_idx >= 0)
            {
                NodeTypeVariants &parent_link_node = nodes_variants_[parent_idx];

                const Mat6<double> Ia = IA(link_node) -
                                        U(link_node) * D_inv(link_node) * U(link_node).transpose();

                const SVec<double> pa = pA(link_node) + Ia * velocityProduct(link_node) +
                                        U(link_node) * D_inv(link_node) * u(link_node);

                pA(parent_link_node) += Xup(link_node).inverseTransformForceVector(pa);
            }
        }

        // Forward Pass - Joint accelerations
        for (NodeTypeVariants &link_node : nodes_variants_)
        {
            const int vel_idx = velocityIndex(link_node);
            const int num_vel = numVelocities(link_node);
            const int parent_idx = parentIndex(link_node); 
            const auto joint = getJoint(link_node);

            SVec<double> a_temp;
            if (parent_idx >= 0)
            {
                NodeTypeVariants &parent_link_node = nodes_variants_[parent_idx];
                a_temp = Xup(link_node).transformMotionVector(acceleration(parent_link_node)) +
                         velocityProduct(link_node);
            }
            else
            {
                a_temp = Xup(link_node).transformMotionVector(-gravity_) +
                         velocityProduct(link_node);
            }
            qdd.segment(vel_idx, num_vel) =
                D_inv(link_node) * (u(link_node) - U(link_node).transpose() * a_temp);

            acceleration(link_node) = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);  
        }

        return qdd;
    }

    void ReflectedInertiaTreeModel::updateArticulatedBodies()
    {
        if (articulated_bodies_updated_)
            return;

        forwardKinematics();

        // Forward pass
        for (NodeTypeVariants &link_node : nodes_variants_)
        {
            // link_node.IA_ = link_node.I_;
            resetArticulatedInertia(link_node);
        }

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)nodes_variants_.size() - 1; i >= 0; i--)
        {
            NodeTypeVariants &link_node = nodes_variants_[i];
            const auto joint = getJoint(link_node);

            const int vel_idx = velocityIndex(link_node);   
            const int num_vel = numVelocities(link_node);
            const int parent_index = parentIndex(link_node);

            U(link_node) = IA(link_node) * joint->S();
            const DMat<double> D = joint->S().transpose() * U(link_node) +
                                   reflected_inertia_.block(vel_idx, vel_idx, num_vel, num_vel);
            D_inv(link_node) = D.inverse();

            // Articulated body inertia recursion
            if (parent_index >= 0)
            {
                NodeTypeVariants &parent_link_node = nodes_variants_[parent_index];
                const DMat<double> Ia = IA(link_node) -
                                        U(link_node) * D_inv(link_node) * U(link_node).transpose();
                IA(parent_link_node) += Xup(link_node).inverseTransformSpatialInertia(Ia);
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
