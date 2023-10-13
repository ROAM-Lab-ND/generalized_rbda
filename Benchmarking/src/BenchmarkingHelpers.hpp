#ifndef GRBDA_BENCHMARK_HELPERS_H
#define GRBDA_BENCHMARK_HELPERS_H

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

namespace grbda
{
    namespace BenchmarkHelpers
    {

        using RigidBodyTreePtr = std::shared_ptr<RigidBodyTreeModel<>>;
        using ReflectedInertiaTreePtr = std::shared_ptr<ReflectedInertiaTreeModel<>>;
        using TreeModelPtr = std::shared_ptr<TreeModel<>>;

        inline bool setRandomStates(ClusterTreeModel<> &cluster_model,
                                    std::vector<RigidBodyTreePtr> rigid_body_models = {},
                                    std::vector<ReflectedInertiaTreePtr> ref_inertia_models = {})
        {

            ModelState<> model_state;
            DVec<double> independent_joint_pos = DVec<double>::Zero(0);
            DVec<double> independent_joint_vel = DVec<double>::Zero(0);
            DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
            DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

            for (const auto &cluster : cluster_model.clusters())
            {
                JointState<> joint_state = cluster->joint_->randomJointState();
                if (joint_state.position.hasNaN())
                {
                    return true;
                }

                JointState<> spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

                DVec<double> independent_joint_pos_i;
                DVec<double> independent_joint_vel_i;
                if (cluster->joint_->type() == ClusterJointTypes::TelloHipDifferential ||
                    cluster->joint_->type() == ClusterJointTypes::TelloKneeAnkleDifferential)
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

            for (ReflectedInertiaTreePtr model : ref_inertia_models)
                model->setIndependentStates(independent_joint_pos, independent_joint_vel);

            return false;
        }

        inline bool setRandomStates(ClusterTreeModel<> &cluster_model,
                                    RigidBodyTreePtr rigid_body_model,
                                    ReflectedInertiaTreePtr reflected_inertia_model)
        {
            return setRandomStates(cluster_model,
                                   std::vector<RigidBodyTreePtr>{rigid_body_model},
                                   std::vector<ReflectedInertiaTreePtr>{reflected_inertia_model});
        }

        struct TrajectoryParameters
        {
            double A;        // amplitude
            double omega;    // frequency
            double phi;      // phase
            double dt;       // time step
            double duration; // duration of trajectory
        };

        struct TrajectoryPoint
        {
            double p; // position
            double v; // velocity
            double a; // acceleration
        };

        struct Trajectory
        {
            // f(t) = A * sin(omega * t + phi)
            // f'(t) = A * omega * cos(omega * t + phi)
            // f''(t) = -A * omega^2 * sin(omega * t + phi)

            Trajectory(const TrajectoryParameters &params)
                : A_(params.A), omega_(params.omega), phi_(params.phi),
                  dt_(params.dt), duration_(params.duration) {}

            TrajectoryPoint at(double t) const
            {
                TrajectoryPoint point;
                point.p = A_ * sin(omega_ * t + phi_);
                point.v = A_ * omega_ * cos(omega_ * t + phi_);
                point.a = -A_ * omega_ * omega_ * sin(omega_ * t + phi_);
                return point;
            }

            const double A_;
            const double omega_;
            const double phi_;

            const double dt_;
            const double duration_;
        };

        class OpenLoopSimulatorBase
        {
        public:
            OpenLoopSimulatorBase(const ClusterTreeModel<> &model,
                                  const TrajectoryPoint &x0,
                                  const double dt)
                : model_(model), x0_(x0), dt_(dt) {}

            void run(const std::vector<DVec<double>> &tau_trajectory)
            {
                // Set initial model state
                ModelState<> model_state;
                for (const auto &cluster : model_.clusters())
                {
                    JointState<> joint_state;
                    joint_state.position = DVec<double>::Constant(cluster->num_positions_, x0_.p);
                    joint_state.velocity = DVec<double>::Constant(cluster->num_velocities_, x0_.v);
                    model_state.push_back(joint_state);
                }
                model_.setState(model_state);

                // Simulate forward in time using the provided torque trajectory
                double t = 0;
                for (const DVec<double> &tau : tau_trajectory)
                {
                    preIntegrationCallback(t, model_state, tau);
                    model_state = rk4IntegrateModelState(model_state, tau);
                    model_.setState(model_state);
                    postIntegrationCallback(t, model_state, tau);
                    t += dt_;
                }
            }

        protected:
            virtual void preIntegrationCallback(double t, const ModelState<> &state,
                                                const DVec<double> &tau) {}
            virtual void postIntegrationCallback(double t, const ModelState<> &state,
                                                const DVec<double> &tau) {}

            ModelState<> eulerIntegrateModelState(ModelState<> model_state,
                                                  const DVec<double> &qdd, const double dt)
            {
                for (size_t i = 0; i < model_state.size(); i++)
                {
                    const auto &cluster = model_.cluster(i);
                    if (cluster->joint_->type() == ClusterJointTypes::Free)
                    {
                        continue;
                    }

                    // Euler integration
                    JointState<double> &joint_state = model_state[i];
                    joint_state.position += dt * joint_state.velocity;
                    joint_state.velocity += dt * qdd.segment(cluster->velocity_index_,
                                                             cluster->num_velocities_);
                }

                return model_state;
            }

            ModelState<> rk4IntegrateModelState(ModelState<> model_state, const DVec<double> &tau)
            {
                // Compute k1
                DVec<double> k1 = model_.forwardDynamics(tau);

                // Compute k2
                ModelState<> model_state_k2 = eulerIntegrateModelState(model_state, k1, dt_ / 2.);
                model_.setState(model_state_k2);
                DVec<double> k2 = model_.forwardDynamics(tau);

                // Compute k3
                ModelState<> model_state_k3 = eulerIntegrateModelState(model_state, k2, dt_ / 2.);
                model_.setState(model_state_k3);
                DVec<double> k3 = model_.forwardDynamics(tau);

                // Compute k4
                ModelState<> model_state_k4 = eulerIntegrateModelState(model_state, k3, dt_);
                model_.setState(model_state_k4);
                DVec<double> k4 = model_.forwardDynamics(tau);

                // Compute qdd
                DVec<double> qdd = (k1 + 2. * k2 + 2. * k3 + k4) / 6.;

                // Integrate
                for (size_t i = 0; i < model_state.size(); i++)
                {
                    const auto &cluster = model_.cluster(i);
                    if (cluster->joint_->type() == ClusterJointTypes::Free)
                    {
                        continue;
                    }

                    JointState<double> &joint_state = model_state[i];
                    const JointState<double> &joint_state_k2 = model_state_k2[i];
                    const JointState<double> &joint_state_k3 = model_state_k3[i];
                    const JointState<double> &joint_state_k4 = model_state_k4[i];

                    DVec<double> qd = (joint_state.velocity + 2. * joint_state_k2.velocity +
                                       2. * joint_state_k3.velocity + joint_state_k4.velocity) /
                                      6.;

                    // Euler integration
                    joint_state.position += dt_ * qd;
                    joint_state.velocity += dt_ * qdd.segment(cluster->velocity_index_,
                                                              cluster->num_velocities_);
                }

                return model_state;
            }

            ClusterTreeModel<> model_;

            const TrajectoryPoint x0_;
            const double dt_;
        };

    }
}

#endif // GRBDA_BENCHMARK_HELPERS_H
