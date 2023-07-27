/*! @file ClusterTreeModel.cpp
 *
 */

#include "ClusterTreeModel.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;
    using namespace ClusterNodeVisitors;

    void ClusterTreeModel::contactJacobians()
    {
        forwardKinematics();

        for (ContactPoint &cp : contact_points_)
        {
            const size_t &i = cp.body_index_;

            const Body &body_i = body(i);
            const NodeType &cluster_i = getNodeContainingBody(body_i.index_);
            const int &subindex_within_cluster_i = body_i.sub_index_within_cluster_;

            const SpatialTransform &X = Xa(cluster_i)[subindex_within_cluster_i];
            const Mat3<double> &R_link_to_world = X.getRotation().transpose();
            Mat6<double> Xout = createSXform(R_link_to_world, cp.local_offset_);

            int j = (int)i;
            while (j > -1)
            {
                const Body &body_j = body(j);
                const NodeType &cluster_j = getNodeContainingBody(body_j.index_);
                const int &subindex_within_cluster_j = body_j.sub_index_within_cluster_;
                const int vel_idx = velocityIndex(cluster_j);
                const int num_vel = numVelocities(cluster_j);

                const D6Mat<double> S =
                    motionSubspace(cluster_j).middleRows<6>(6 * subindex_within_cluster_j);
                cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

                const Mat6<double> Xup_j = Xup(cluster_j)[subindex_within_cluster_j].toMatrix();
                Xout = Xout * Xup_j;

                j = body_j.cluster_ancestor_index_;
            }
        }
    }

    DVec<double> ClusterTreeModel::inverseDynamics(const DVec<double> &qdd)
    {
        return recursiveNewtonEulerAlgorithm(qdd);
    }

    DVec<double> ClusterTreeModel::forwardDynamics(const DVec<double> &tau)
    {
        DVec<double> qdd = DVec<double>::Zero(getNumDegreesOfFreedom());

        // Forward dynamics via Articulated Body Algorithm
#ifdef TIMING_STATS
        timing_statistics_.zero();
        timer_.start();
#endif
        forwardKinematics();
#ifdef TIMING_STATS
        timing_statistics_.forward_kinematics_time = timer_.getMs();
        timer_.start();
#endif
        updateArticulatedBodies();
#ifdef TIMING_STATS
        timing_statistics_.update_articulated_bodies_time = timer_.getMs();
        timer_.start();
#endif

        // Forward Pass - Articulated body bias force
        for (NodeType &cluster : nodes_)
        {
            updateArticulatedBiasForce(cluster);
        }
#ifdef TIMING_STATS
        timing_statistics_.forward_pass1_time = timer_.getMs();
        timer_.start();
#endif

        // Account for external forces in bias force
        for (int cluster_index : indices_of_nodes_experiencing_external_forces_)
        {
            NodeType &cluster_var = nodes_[cluster_index];
            pA(cluster_var) -=
                Xa(cluster_var).transformExternalForceVector(externalForce(cluster_var));
        }
#ifdef TIMING_STATS
        timing_statistics_.external_force_time = timer_.getMs();
        timer_.start();
#endif

        // Backward pass - Gauss principal of least constraint
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            NodeType &cluster = nodes_[i];
            const int vel_idx = velocityIndex(cluster);
            const int num_vel = numVelocities(cluster);
            const int parent_index = parentIndex(cluster);
            const auto joint = getJoint(cluster);

            u(cluster) = tau.segment(vel_idx, num_vel) - joint->S().transpose() * pA(cluster);
            D_inv_u(cluster) = D_inv(cluster).solve(u(cluster));

            // Articulated body bias force recursion
            if (parent_index >= 0)
            {
                NodeType &parent_cluster = nodes_[parent_index];

                const DVec<double> pa = pA(cluster) + Ia(cluster) * velocityProduct(cluster) +
                                        U(cluster) * D_inv_u(cluster);

                pA(parent_cluster) += Xup(cluster).inverseTransformForceVector(pa);
            }
        }
#ifdef TIMING_STATS
        timing_statistics_.backward_pass_time = timer_.getMs();
        timer_.start();
#endif

        // Forward Pass - Joint accelerations
        for (NodeType &cluster : nodes_)
        {
            const int vel_idx = velocityIndex(cluster);
            const int num_vel = numVelocities(cluster);
            const int parent_index = parentIndex(cluster);
            const auto joint = getJoint(cluster);

            DVec<double> a_temp;
            if (parent_index >= 0)
            {
                NodeType &parent_cluster = nodes_[parent_index];
                a_temp = Xup(cluster).transformMotionVector(acceleration(parent_cluster)) +
                         velocityProduct(cluster);
            }
            else
            {
                a_temp = Xup(cluster).transformMotionVector(-gravity_) + velocityProduct(cluster);
            }
            qdd.segment(vel_idx, num_vel) = D_inv_u(cluster) - D_inv_UT(cluster) * a_temp;
            acceleration(cluster) = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }
#ifdef TIMING_STATS
        timing_statistics_.forward_pass2_time = timer_.getMs();
#endif

        return qdd;
    }

    void ClusterTreeModel::updateArticulatedBodies()
    {
        if (articulated_bodies_updated_)
            return;

        forwardKinematics();

        // Forward pass
#ifdef TIMING_STATS
        double start_time_IA = timer_.getMs();
#endif
        for (NodeType &cluster : nodes_)
        {
            resetArticulatedInertia(cluster);
        }
#ifdef TIMING_STATS
        timing_statistics_.reset_IA_time += timer_.getMs() - start_time_IA;
#endif

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            NodeType &cluster = nodes_[i];
            const int parent_index = parentIndex(cluster);
            const auto joint = getJoint(cluster);
#ifdef TIMING_STATS
            double start_time_D = timer_.getMs();
#endif
            U(cluster) = IA(cluster) * joint->S();
            const DMat<double> D = joint->S().transpose() * U(cluster);
            updateDinv(cluster, D);
            D_inv_UT(cluster) = D_inv(cluster).solve(U(cluster).transpose());
#ifdef TIMING_STATS
            timing_statistics_.update_and_solve_D_time += timer_.getMs() - start_time_D;
#endif

            // Articulated body inertia recursion
            if (parent_index >= 0)
            {
#ifdef TIMING_STATS
                double start_time_Ia = timer_.getMs();
#endif
                NodeType &parent_cluster = nodes_[parent_index];
                Ia(cluster) = IA(cluster) - U(cluster) * D_inv_UT(cluster);
                IA(parent_cluster) += Xup(cluster).inverseTransformSpatialInertia(Ia(cluster));
#ifdef TIMING_STATS
                timing_statistics_.invert_xform_spatial_inertia_time += timer_.getMs() - start_time_Ia;
#endif
            }
        }

        articulated_bodies_updated_ = true;
    }

    double ClusterTreeModel::applyTestForce(const string &cp_name,
                                            const Vec3<double> &force_ics_at_contact,
                                            DVec<double> &dstate_out)
    {
        return applyLocalFrameTestForceAtContactPoint(force_ics_at_contact, cp_name, dstate_out);
    }

    double
    ClusterTreeModel::applyLocalFrameTestForceAtContactPoint(const Vec3<double> &force,
                                                             const std::string &contact_point_name,
                                                             DVec<double> &dstate_out)
    {
        const int contact_point_index = contact_name_to_contact_index_.at(contact_point_name);
        const ContactPoint &contact_point = contact_points_[contact_point_index];

        forwardKinematics();
        updateArticulatedBodies();
        updateForcePropagators();
        updateQddEffects();

        dstate_out = DVec<double>::Zero(getNumDegreesOfFreedom());

        DVec<double> f = localCartesianForceAtPointToWorldPluckerForceOnCluster(force,
                                                                                contact_point);
        double lambda_inv = 0.;

        // from tips to base
        int j = getIndexOfClusterContainingBody(contact_point.body_index_);
        while (j > -1)
        {
            NodeType &cluster = nodes_[j];
            const int vel_idx = velocityIndex(cluster);
            const int num_vel = numVelocities(cluster);
            const auto joint = getJoint(cluster);

            const DVec<double> tmp = joint->S().transpose() * f;
            lambda_inv += tmp.dot(D_inv(cluster).solve(tmp));

            dstate_out += qddSubtree(cluster) * D_inv(cluster).solve(tmp);

            f = ChiUp(cluster).transpose() * f;

            j = parentIndex(cluster);
        }

        return lambda_inv;
    }

    void ClusterTreeModel::updateForcePropagators()
    {
        if (force_propagators_updated_)
            return;

        updateArticulatedBodies();

        for (NodeType &cluster : nodes_)
        {
            const auto joint = getJoint(cluster);
            const DMat<double> X = Xup(cluster).toMatrix();
            ChiUp(cluster) = X - joint->S() * D_inv_UT(cluster) * X;
        }

        force_propagators_updated_ = true;
    }

    void ClusterTreeModel::updateQddEffects()
    {
        if (qdd_effects_updated_)
            return;

        updateForcePropagators();

        for (NodeType &cluster : nodes_)
        {
            const int vel_idx = velocityIndex(cluster);
            const int num_vel = numVelocities(cluster);
            const int parent_index = parentIndex(cluster);
            const auto joint = getJoint(cluster);

            qddSubtree(cluster).middleRows(vel_idx, num_vel).setIdentity();

            // Compute Psi
            const DMat<double> S = joint->S();
            const DMat<double> Psi =
                S.transpose().completeOrthogonalDecomposition().pseudoInverse();

            DMat<double> F =
                (ChiUp(cluster).transpose() - Xup(cluster).toMatrix().transpose()) * Psi;

            int j = parentIndex(cluster);
            while (j > -1)
            {
                NodeType &parent_cluster = nodes_[j];
                const auto parent_joint = getJoint(parent_cluster);

                qddSubtree(parent_cluster).middleRows(vel_idx, num_vel) = F.transpose() * parent_joint->S();

                F = ChiUp(parent_cluster).transpose() * F;
                j = parentIndex(parent_cluster);
            }
        }

        qdd_effects_updated_ = true;
    }

} // namespace grbda
