/*! @file ClusterTreeModel.cpp
 *
 */

#include "ClusterTreeModel.h"

using namespace ori;
using namespace spatial;

DVec<double> ClusterTreeModel::inverseDyamics(const DVec<double> &qdd)
{
    return recursiveNewtonEulerAlgorithm(qdd);
}

DVec<double> ClusterTreeModel::forwardDynamics(const DVec<double> &tau)
{
    // Forward dynamics via Articulated Body Algorithm
    forwardKinematics();
    updateArticulatedBodies();

    DVec<double> qdd = DVec<double>::Zero(getNumDegreesOfFreedom());

    // Forward Pass - Articulated body bias force
    for (auto &cluster : cluster_nodes_)
    {
        cluster->pA_ = generalForceCrossProduct(cluster->v_, DVec<double>(cluster->I_ * cluster->v_));
    }

    // Account for external forces in bias force
    for (int cluster_index : indices_of_nodes_experiencing_external_forces_)
    {
        auto &cluster = cluster_nodes_[cluster_index];
        cluster->pA_ -= cluster->Xa_.transformExternalForceVector(cluster->f_ext_);
    }

    // Backward pass - Gauss principal of least constraint
    for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
    {
        auto &cluster = cluster_nodes_[i];
        const int vel_idx = cluster->velocity_index_;
        const int num_vel = cluster->num_velocities_;
        const auto joint = cluster->joint_;

        cluster->u_ = tau.segment(vel_idx, num_vel) - joint->S().transpose() * cluster->pA_;

        // Articulated body bias force recursion
        if (cluster->parent_index_ >= 0)
        {
            auto parent_cluster = cluster_nodes_[cluster->parent_index_];

            DMat<double> Ia =
                cluster->IA_ - cluster->U_ * cluster->D_inv_ * cluster->U_.transpose();

            DVec<double> pa =
                cluster->pA_ + Ia * cluster->c_ + cluster->U_ * cluster->D_inv_ * cluster->u_;

            parent_cluster->pA_ += cluster->Xup_.inverseTransformForceVector(pa);
        }
    }

    // Forward Pass - Joint accelerations
    for (auto &cluster : cluster_nodes_)
    {
        const int vel_idx = cluster->velocity_index_;
        const int num_vel = cluster->num_velocities_;
        const auto joint = cluster->joint_;

        DVec<double> a_temp;
        if (cluster->parent_index_ >= 0)
        {
            const auto parent_cluster = cluster_nodes_[cluster->parent_index_];
            a_temp = cluster->Xup_.transformMotionVector(parent_cluster->a_) + cluster->c_;
        }
        else
        {
            a_temp = cluster->Xup_.transformMotionVector(-gravity_) + cluster->c_;
        }
        qdd.segment(vel_idx, num_vel) =
            cluster->D_inv_ * (cluster->u_ - cluster->U_.transpose() * a_temp);
        cluster->a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
    }

    return qdd;
}

void ClusterTreeModel::updateArticulatedBodies()
{
    if (articulated_bodies_updated_)
        return;

    forwardKinematics();

    // Forward pass
    for (auto &cluster : cluster_nodes_)
    {
        cluster->IA_ = cluster->I_;
    }

    // Backward pass (Gauss principal of least constraint)
    for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
    {
        auto &cluster = cluster_nodes_[i];
        const auto joint = cluster->joint_;
        cluster->U_ = cluster->IA_ * joint->S();
        cluster->D_inv_ = (joint->S().transpose() * cluster->U_).inverse();

        // Articulated body inertia recursion
        if (cluster->parent_index_ >= 0)
        {
            auto parent_cluster = cluster_nodes_[cluster->parent_index_];
            DMat<double> Ia =
                cluster->IA_ - cluster->U_ * cluster->D_inv_ * cluster->U_.transpose();
            parent_cluster->IA_ += cluster->Xup_.inverseTransformSpatialInertia(Ia);
        }
    }

    articulated_bodies_updated_ = true;
}

double
ClusterTreeModel::applyLocalFrameTestForceAtConactPoint(const Vec3<double> &force,
                                                        const std::string &contact_point_name,
                                                        DVec<double> &dstate_out)
{
    contact_name_to_contact_index_.checkForKey(contact_point_name);
    const int contact_point_index = contact_name_to_contact_index_[contact_point_name];
    const ContactPoint &contact_point = contact_points_[contact_point_index];

    forwardKinematics();
    updateArticulatedBodies();
    updateForcePropagators();
    updateQddEffects();

    dstate_out = DVec<double>::Zero(getNumDegreesOfFreedom());

    DVec<double> f = localCartesianForceAtPointToWorldPluckerForceOnCluster(force, contact_point);
    Vec1<double> lambda_inv = Vec1<double>::Zero();

    // from tips to base
    int j = getIndexOfClusterContainingBody(contact_point.body_index_);
    while (j > -1)
    {
        const auto &cluster = cluster_nodes_[j];
        const int vel_idx = cluster->velocity_index_;
        const int num_vel = cluster->num_velocities_;
        const auto joint = cluster->joint_;

        D1Mat<double> tmp = D1Mat<double>::Zero(1, num_vel);
        tmp = f.transpose() * joint->S();
        lambda_inv += tmp * cluster->D_inv_ * tmp;

        dstate_out +=
            cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd *
            cluster->D_inv_ * tmp.transpose();

        f = cluster->ChiUp_.transpose() * f;

        j = cluster->parent_index_;
    }

    return lambda_inv[0];
}

void ClusterTreeModel::updateForcePropagators()
{
    if (force_propagators_updated_)
        return;

    updateArticulatedBodies();

    for (auto &cluster : cluster_nodes_)
    {
        const auto joint = cluster->joint_;
        cluster->ChiUp_ =
            cluster->Xup_.toMatrix() -
            joint->S() * cluster->D_inv_ * cluster->U_.transpose() * cluster->Xup_.toMatrix();
    }

    force_propagators_updated_ = true;
}

void ClusterTreeModel::updateQddEffects()
{
    if (qdd_effects_updated_)
        return;

    updateForcePropagators();

    for (auto &cluster : cluster_nodes_)
    {
        const int vel_idx = cluster->velocity_index_;
        const int num_vel = cluster->num_velocities_;
        const auto joint = cluster->joint_;

        cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
            .middleRows(vel_idx, num_vel)
            .setIdentity();
        DVec<double> F =
            (cluster->ChiUp_.transpose() - cluster->Xup_.toMatrix().transpose()) * joint->Psi();

        int j = cluster->parent_index_;
        while (j > -1)
        {
            auto parent_cluster = cluster_nodes_[j];
            const auto parent_joint = cluster->joint_;

            parent_cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .middleRows(vel_idx, num_vel) = parent_joint->S().transpose() * F;

            F = parent_cluster->ChiUp_.transpose() * F;
            j = parent_cluster->parent_index_;
        }
    }

    qdd_effects_updated_ = true;
}
