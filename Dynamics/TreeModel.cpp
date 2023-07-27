#include "TreeModel.h"

#include "ClusterTreeModel.h"
#include "RigidBodyTreeModel.h"
#include "ReflectedInertiaTreeModel.h"

// TODO(@MatthewChignoli): worry about the external force visitors later. And same with the contact jacobians I guess

namespace grbda
{

    template <typename Derived>
    void TreeModel<Derived>::forwardKinematics()
    {
        if (kinematics_updated_)
            return;

        for (NodeTypeVariants &node_var : nodes_variants_)
        {
            const int parent_index = parentIndex(node_var);
            if (parent_index >= 0)
            {
                const NodeTypeVariants &parent_node_var = nodes_variants_[parent_index];
                nodeKinematics(node_var, velocity(parent_node_var), Xa(parent_node_var));
            }
            else
            {
                nodeKinematics(node_var);
            }
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
            const NodeTypeVariants &node = getNodeVariantContainingBody(cp.body_index_);

            const SpatialTransform Xa = getAbsoluteTransformForBody(node, body);
            const SVec<double> v_body = getVelocityForBody(node, body);

            cp.position_ = Xa.inverseTransformPoint(cp.local_offset_);
            cp.velocity_ = spatialToLinearVelocity(Xa.inverseTransformMotionVector(v_body),
                                                   cp.position_);
        }
    }

    // TODO(@MatthewChignoli): Stuff with backward passes is tough, I think this is why Pinocchio uses the data structure that it does, so that it can pass them separately...
    template <typename Derived>
    void TreeModel<Derived>::compositeRigidBodyAlgorithm()
    {
        if (mass_matrix_updated_)
            return;

        forwardKinematics();

        // Forward Pass
        for (NodeTypeVariants &node_var : nodes_variants_)
            resetCompositeRigidBodyInertia(node_var);

        for (int i = (int)nodes_variants_.size() - 1; i >= 0; i--)
        {
            NodeTypeVariants &node_var_i = nodes_variants_[i];
            const int vel_idx_i = velocityIndex(node_var_i);
            const int num_vel_i = numVelocities(node_var_i);
            const int parent_index = parentIndex(node_var_i);

            if (parent_index >= 0)
            {
                NodeTypeVariants &parent_node_var = nodes_variants_[parent_index];
                compositeInertia(parent_node_var) +=
                    Xup(node_var_i).inverseTransformSpatialInertia(compositeInertia(node_var_i));
            }

            DMat<double> &Ic = compositeInertia(node_var_i);
            const DMat<double> &S = motionSubspace(node_var_i);
            DMat<double> F = Ic * S;

            H_.block(vel_idx_i, vel_idx_i, num_vel_i, num_vel_i) = S.transpose() * F;

            int j = i;
            while (parentIndex(nodes_variants_[j]) > -1)
            {
                F = Xup(nodes_variants_[j]).inverseTransformForceSubspace(F);

                j = parentIndex(nodes_variants_[j]);
                const int vel_idx_j = velocityIndex(nodes_variants_[j]);
                const int num_vel_j = numVelocities(nodes_variants_[j]);

                H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j) =
                    F.transpose() * motionSubspace(nodes_variants_[j]);
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
        for (NodeTypeVariants &node_var : nodes_variants_)
        {
            const int vel_idx = velocityIndex(node_var);
            const int num_vel = numVelocities(node_var);
            const int parent_index = parentIndex(node_var);

            DVec<double> parent_accel;
            if (parent_index >= 0)
            {
                NodeTypeVariants &parent_node_var = nodes_variants_[parent_index];
                parent_accel = acceleration(parent_node_var);
            }
            else
            {
                parent_accel = -gravity_;
            }

            acceleration(node_var) = Xup(node_var).transformMotionVector(parent_accel) +
                                     motionSubspace(node_var) * qdd.segment(vel_idx, num_vel) +
                                     velocityProduct(node_var);

            setForceFromAcceleration(node_var);
        }

        // Account for external forces in bias force
        for (int index : indices_of_nodes_experiencing_external_forces_)
        {
            NodeTypeVariants &node = nodes_variants_[index];
            netForce(node) -= Xa(node).transformExternalForceVector(externalForce(node));
        }

        // Backward Pass
        for (int i = (int)nodes_variants_.size() - 1; i >= 0; i--)
        {
            NodeTypeVariants &node_var = nodes_variants_[i];
            const int vel_idx = velocityIndex(node_var);
            const int num_vel = numVelocities(node_var);
            const int parent_index = parentIndex(node_var);

            const DVec<double> &net_force = netForce(node_var);

            tau.segment(vel_idx, num_vel) = motionSubspace(node_var).transpose() * net_force;

            if (parent_index >= 0)
            {
                NodeTypeVariants &parent_node_var = nodes_variants_[parent_index];
                netForce(parent_node_var) += Xup(node_var).inverseTransformForceVector(net_force);
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
            externalForce(nodes_variants_[index]).setZero();

        // Apply forces to nodes
        indices_of_nodes_experiencing_external_forces_.clear();
        for (const auto &force_and_body_index : force_and_body_index_pairs)
        {
            const SVec<double> &force = force_and_body_index.force_;
            const int body_index = force_and_body_index.index_;

            const auto &body = getBody(body_index);
            NodeTypeVariants &node = getNodeVariantContainingBody(body_index);
            applyForceToBody(node, force, body);

            // Add index to vector if vector does not already contain this cluster
            if (!vectorContainsIndex(indices_of_nodes_experiencing_external_forces_, index(node)))
                indices_of_nodes_experiencing_external_forces_.push_back(index(node));
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
