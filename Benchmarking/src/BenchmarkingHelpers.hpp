#ifndef GRBDA_BENCHMARK_HELPERS_H
#define GRBDA_BENCHMARK_HELPERS_H

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

namespace grbda
{
    namespace BenchmarkHelpers
    {

        using RigidBodyTreePtr = std::shared_ptr<RigidBodyTreeModel>;
        using ReflectedInertiaTreePtr = std::shared_ptr<ReflectedInertiaTreeModel>;

        inline bool setRandomStates(ClusterTreeModel &cluster_model,
                                    std::vector<RigidBodyTreePtr> rigid_body_models,
                                    std::vector<ReflectedInertiaTreePtr> reflected_inertia_models)
        {

            ModelState model_state;
            DVec<double> independent_joint_pos = DVec<double>::Zero(0);
            DVec<double> independent_joint_vel = DVec<double>::Zero(0);
            DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
            DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

            for (const auto &cluster : cluster_model.clusters())
            {
                JointState joint_state = cluster->joint_->randomJointState();
                if (joint_state.position.hasNaN())
                {
                    return true;
                }

                JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

                DVec<double> independent_joint_pos_i;
                DVec<double> independent_joint_vel_i;
                if (cluster->joint_->type() == GeneralizedJointTypes::TelloHipDifferential ||
                    cluster->joint_->type() == GeneralizedJointTypes::TelloKneeAnkleDifferential)
                {
                    independent_joint_pos_i = spanning_joint_state.position.tail<2>();
                    independent_joint_vel_i = spanning_joint_state.velocity.tail<2>();
                }
                else
                {
                    if (joint_state.position.isSpanning() || joint_state.velocity.isSpanning())
                        throw std::runtime_error("Initializing reflected inertia model requires all independent coordinates");
                    independent_joint_pos_i = joint_state.position;
                    independent_joint_vel_i = joint_state.velocity;
                }

                independent_joint_pos = appendEigenVector(independent_joint_pos,
                                                          independent_joint_pos_i);
                independent_joint_vel = appendEigenVector(independent_joint_vel,
                                                          independent_joint_vel_i);

                spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                       spanning_joint_state.position);
                spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                       spanning_joint_state.velocity);

                model_state.push_back(joint_state);
            }

            cluster_model.setState(model_state);

            for (RigidBodyTreePtr model : rigid_body_models)
                model->setState(spanning_joint_pos, spanning_joint_vel);

            for (ReflectedInertiaTreePtr model : reflected_inertia_models)
                model->setIndependentStates(independent_joint_pos, independent_joint_vel);

            return false;
        }

        inline bool setRandomStates(ClusterTreeModel &cluster_model,
                                    RigidBodyTreePtr rigid_body_model,
                                    ReflectedInertiaTreePtr reflected_inertia_model)
        {
            return setRandomStates(cluster_model,
                                   std::vector<RigidBodyTreePtr>{rigid_body_model},
                                   std::vector<ReflectedInertiaTreePtr>{reflected_inertia_model});
        }

    }
}

#endif // GRBDA_BENCHMARK_HELPERS_H
