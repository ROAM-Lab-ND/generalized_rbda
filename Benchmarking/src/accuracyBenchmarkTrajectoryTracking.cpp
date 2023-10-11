#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

struct TrajectoryPoint
{
    double p; // position
    double v; // velocity
    double a; // acceleration
};

// TODO(@MatthewChignoli): Use casadi and lambda functions to generate the trajectory
TrajectoryPoint trajectory(double t)
{
    // f(t) = A * sin(omega * t + phi)
    // f'(t) = A * omega * cos(omega * t + phi)
    // f''(t) = -A * omega^2 * sin(omega * t + phi)

    const double A = 0.1;
    const double omega = 0.5 * (2. * M_PI);
    const double phi = 0.;

    TrajectoryPoint point;
    point.p = A * sin(omega * t + phi);
    point.v = A * omega * cos(omega * t + phi);
    point.a = -A * omega * omega * sin(omega * t + phi);

    return point;
}

class TorqueTrajectoryGenerator
{
public:
    typedef std::vector<DVec<double>> TauTrajectory;

    TorqueTrajectoryGenerator(std::ofstream &trajectory_file,
                              ClusterTreeModel<> model_cl,
                              const std::string contact_point,
                              const double duration, const double dt)
        : dt_(dt), file_(trajectory_file), contact_point_(contact_point)
    {
        ReflectedInertiaTreeModel<> model_rf_diag(model_cl, RotorInertiaApproximation::DIAGONAL);
        ReflectedInertiaTreeModel<> model_rf_none(model_cl, RotorInertiaApproximation::NONE);

        const int nq = model_cl.getNumPositions();
        const int nv = model_cl.getNumDegreesOfFreedom();

        for (double t = 0; t <= duration; t += dt_)
        {
            // Set State
            const TrajectoryPoint point = trajectory(t);
            DVec<double> q = DVec<double>::Constant(nq, point.p);
            DVec<double> qd = DVec<double>::Constant(nv, point.v);
            DVec<double> qdd = DVec<double>::Constant(nv, point.a);

            setState(model_cl, q, qd);
            setState(model_rf_diag, q, qd);
            setState(model_rf_none, q, qd);

            // Compute Desired Contact Point Position
            model_cl.forwardKinematicsIncludingContactPoints();
            const ContactPoint<double> &cp = model_cl.contactPoint(contact_point_);
            file_ << t << " " << cp.position_.transpose() << std::endl;

            // Compute torque needed to track the trajectory via inverse dynamics
            tau_traj_map_["exact"].push_back(model_cl.inverseDynamics(qdd));
            tau_traj_map_["diag"].push_back(model_rf_diag.inverseDynamics(qdd));
            tau_traj_map_["none"].push_back(model_rf_none.inverseDynamics(qdd));
        }
    }

    const TauTrajectory &getTorqueTrajectory(const std::string &model_name) const
    {
        return tau_traj_map_.at(model_name);
    }

    const double dt_;

private:
    void setState(ReflectedInertiaTreeModel<> &model_rf,
                  const DVec<double> &q, const DVec<double> &qd)
    {
        model_rf.setIndependentStates(q, qd);
    }

    void setState(ClusterTreeModel<> &model_cl,
                  const DVec<double> &q, const DVec<double> &qd)
    {
        ModelState<> model_state;
        for (const auto &cluster : model_cl.clusters())
        {
            JointState<> joint_state;
            joint_state.position = q.segment(cluster->position_index_, cluster->num_positions_);
            joint_state.velocity = qd.segment(cluster->velocity_index_, cluster->num_velocities_);
            model_state.push_back(joint_state);
        }
        model_cl.setState(model_state);
    }

    std::ofstream &file_;
    const std::string contact_point_;
    std::map<std::string, TauTrajectory> tau_traj_map_;
};

class OpenLoopSimulator
{
public:
    OpenLoopSimulator(std::ofstream &trajectory_file,
                      const ClusterTreeModel<> &model,
                      const TrajectoryPoint x0,
                      const std::string contact_point)
        : file_(trajectory_file), model_(model), x0_(x0), contact_point_(contact_point) {}

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
            // Compute the realized Contact Point Position
            model_.forwardKinematicsIncludingContactPoints();
            const ContactPoint<double> &cp = model_.contactPoint(contact_point_);
            file_ << t << " " << cp.position_.transpose() << std::endl;

            // Simulate forward in time using the torque from the approximate model
            model_state = rk4IntegrateModelState(model_state, tau);
            model_.setState(model_state);

            t += dt_;
        }
    }

    double dt_ = 5e-5;

private:
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

    std::ofstream &file_;
    ClusterTreeModel<> model_;
    const TrajectoryPoint x0_;
    const std::string contact_point_;
};

template <typename RobotType>
void runOpenLoopTrajectoryBenchmark(std::ofstream &file, const std::string &contact_point,
                                    double duration, double dt)
{
    // This test demonstrates the important of accounting for the complete closed loop effects of 
    // actuation sub-mechanisms. It does so via the following steps:
    // 1) Given a trajectory q(t), qd(t), qdd(t), use inverse dynamics to compute the torque 
    // trajectory tau(t) needed to accomplish the trajectory. We do so using the exact model as 
    // well as two approximate models
    // 2) Simulate the exact model forward in time using the various torque trajectories
    // 3) Compare the realized 3D trajectories of a contact point on the robot resulting from the 
    // various torque trajectories

    RobotType robot;
    ClusterTreeModel<> model = robot.buildClusterTreeModel();

    TorqueTrajectoryGenerator trajectory_generator(file, model, contact_point, duration, dt);

    OpenLoopSimulator simulator(file, model, trajectory(0), contact_point);
    simulator.dt_ = trajectory_generator.dt_;
    simulator.run(trajectory_generator.getTorqueTrajectory("exact"));
    simulator.run(trajectory_generator.getTorqueTrajectory("diag"));
    simulator.run(trajectory_generator.getTorqueTrajectory("none"));
}

int main()
{
    // Trajectory Benchmark
    std::cout << "\n\n**Starting Trajectory Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyTT_";
    std::ofstream trajectory_file;
    trajectory_file.open(path_to_data + "Robots.csv");
    runOpenLoopTrajectoryBenchmark<MIT_Humanoid_Leg>(trajectory_file, "toe_contact", 5.0, 1e-5);

    // TODO(@MatthewChignoli: Tello joint requires spanning trajectory?
    // runOpenLoopTrajectoryBenchmark<TelloLeg>(trajectory_file, "toe_contact", 5.0, 1e-5);
    trajectory_file.close();

    return 0;
}
