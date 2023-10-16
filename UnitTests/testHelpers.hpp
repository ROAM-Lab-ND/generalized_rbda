#ifndef GRBDA_TEST_HELPERS_H
#define GRBDA_TEST_HELPERS_H

#include "Dynamics/ClusterTreeModel.h"

using namespace grbda;

// TODO(@MatthewChignoli): include other functions in this namespace
namespace TestHelpers
{
    inline ClusterTreeModel<> extractGenericJointModel(const ClusterTreeModel<> &model)
    {
        using namespace ClusterJoints;

        ClusterTreeModel<> generic_model{};

        for (const auto &cluster : model.clusters())
        {
            std::vector<Body<>> bodies;
            std::vector<JointPtr<double>> joints;

            // Register bodies
            for (auto pair : cluster->bodiesAndJoints())
            {
                Body<> body_i = pair.first;
                bodies.push_back(body_i);

                JointPtr<double> joint_i = pair.second;
                joints.push_back(joint_i);

                bool is_base = body_i.parent_index_ == -1;
                std::string parent_name = is_base ? "ground" : model.body(body_i.parent_index_).name_;
                generic_model.registerBody(body_i.name_, body_i.inertia_, parent_name, body_i.Xtree_);
            }

            // Extract Loop Constraint and Append Cluster
            std::shared_ptr<LoopConstraint::Base<double>> constraint = cluster->joint_->cloneLoopConstraint();
            generic_model.appendRegisteredBodiesAsCluster<Generic<>>(cluster->name_, bodies,
                                                                     joints, constraint);
        }

        return generic_model;
    }

    inline DVec<casadi::SX> plus(ClusterJointTypes joint_type,
                                 DVec<casadi::SX> q, DVec<casadi::SX> dq)
    {
        using SX = casadi::SX;

        if (joint_type == ClusterJointTypes::Free)
        {
            const Vec3<SX> pos = q.head<3>();
            const Quat<SX> quat = q.tail<4>();

            const Vec3<SX> dquat = dq.head<3>();
            const Vec3<SX> dpos = dq.tail<3>();

            Vec7<SX> q_plus_dq_vec;

            const Mat3<SX> R = ori::quaternionToRotationMatrix(quat);
            q_plus_dq_vec.head<3>() = pos + R.transpose() * dpos;

            q_plus_dq_vec.template tail<4>() = quat + 0.5 * ori::quatProduct(quat, Quat<casadi::SX>(0, dquat[0], dquat[1], dquat[2]));

            return q_plus_dq_vec;
        }
        else
        {
            return q + dq;
        }
    }

    inline casadi::DMVector plus(const casadi::DMVector q, const casadi::DMVector dq)
    {
        casadi::DMVector q_plus_dq_vec = q;

        int pos_idx = 0;
        int vel_idx = 0;

        if (q.size() != dq.size()) // floating base joint
        {
            DVec<casadi::SX> q_vec(7);
            q_vec << q[0], q[1], q[2], q[3], q[4], q[5], q[6];

            DVec<casadi::SX> dq_vec(6);
            dq_vec << dq[0], dq[1], dq[2], dq[3], dq[4], dq[5];

            DVec<casadi::SX> qfb_plus_dq_vec = plus(ClusterJointTypes::Free, q_vec, dq_vec);

            for (int i = 0; i < 7; i++)
            {
                q_plus_dq_vec[i] = qfb_plus_dq_vec[i];
            }

            pos_idx = 7;
            vel_idx = 6;
        }

        for (int i = pos_idx; i < q.size(); i++)
        {
            q_plus_dq_vec[i] = q[i] + dq[vel_idx];
            vel_idx++;
        }

        return q_plus_dq_vec;
    }

}

#endif // GRBDA_TEST_HELPERS_H
