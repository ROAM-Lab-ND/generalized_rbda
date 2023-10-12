#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

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

class TorqueTrajectoryGenerator
{
public:
    typedef std::vector<DVec<double>> TauTrajectory;

    TorqueTrajectoryGenerator(std::ofstream &trajectory_file,
                              ClusterTreeModel<> model_cl,
                              const std::string contact_point,
                              const Trajectory &trajectory)
        : file_(trajectory_file), contact_point_(contact_point)
    {
        ReflectedInertiaTreeModel<> model_rf_diag(model_cl, RotorInertiaApproximation::DIAGONAL);
        ReflectedInertiaTreeModel<> model_rf_none(model_cl, RotorInertiaApproximation::NONE);

        const int nq = model_cl.getNumPositions();
        const int nv = model_cl.getNumDegreesOfFreedom();

        for (double t = 0; t <= trajectory.duration_; t += trajectory.dt_)
        {
            // Set State
            const TrajectoryPoint point = trajectory.at(t);
            DVec<double> q = DVec<double>::Constant(nq, point.p);
            DVec<double> qd = DVec<double>::Constant(nv, point.v);
            DVec<double> qdd = DVec<double>::Constant(nv, point.a);

            setState(model_cl, q, qd);
            setState(model_rf_diag, q, qd);
            setState(model_rf_none, q, qd);

            // Compute Desired Contact Point Position
            model_cl.forwardKinematicsIncludingContactPoints();
            const Vec3<double> cp_pos = model_cl.contactPoint(contact_point_).position_;
            file_ << trajectory.omega_ << " " << t << " " << cp_pos.transpose() << std::endl;

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
    OpenLoopSimulator(std::ofstream &trajectory_file, const ClusterTreeModel<> &model,
                      const std::string contact_point, const Trajectory &trajectory)
        : file_(trajectory_file), model_(model), contact_point_(contact_point),
          x0_(trajectory.at(0)), dt_(trajectory.dt_), omega_(trajectory.omega_) {}

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
            const Vec3<double> cp_pos = model_.contactPoint(contact_point_).position_;
            file_ << omega_ << " " << t << " " << cp_pos.transpose() << std::endl;

            // Simulate forward in time using the torque from the approximate model
            model_state = rk4IntegrateModelState(model_state, tau);
            model_.setState(model_state);

            t += dt_;
        }
    }

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
    const std::string contact_point_;

    const TrajectoryPoint x0_;
    const double dt_;
    const double omega_;
};

template <typename RobotType>
void runOpenLoopTrajectoryBenchmark(std::ofstream &file, const std::string &contact_point,
                                    double duration, double dt)
{
    // TODO(@MatthewChignoli): Update this description
    
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

    TrajectoryParameters trajectory_params;
    trajectory_params.A = 0.1;
    trajectory_params.phi = 0.;
    trajectory_params.dt = dt;
    trajectory_params.duration = duration;

    for (double omega = 0.2; omega <= 2.0; omega += 0.2)
    {
        trajectory_params.omega = omega * 2. * M_PI;
        Trajectory trajectory(trajectory_params);
        TorqueTrajectoryGenerator trajectory_generator(file, model, contact_point, trajectory);

        OpenLoopSimulator simulator(file, model, contact_point, trajectory);
        simulator.run(trajectory_generator.getTorqueTrajectory("exact"));
        simulator.run(trajectory_generator.getTorqueTrajectory("diag"));
        simulator.run(trajectory_generator.getTorqueTrajectory("none"));
    }
}

int main()
{
    // Trajectory Benchmark
    std::cout << "\n\n**Starting Trajectory Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyTT_";
    std::ofstream trajectory_file;
    trajectory_file.open(path_to_data + "Robots.csv");
    runOpenLoopTrajectoryBenchmark<MIT_Humanoid_Leg>(trajectory_file, "toe_contact", 6.0, 2e-3);

    // TODO(@MatthewChignoli: Tello joint requires spanning trajectory?
    // runOpenLoopTrajectoryBenchmark<TelloLeg>(trajectory_file, "toe_contact", 5.0, 1e-5);

    trajectory_file.close();

    return 0;
}
