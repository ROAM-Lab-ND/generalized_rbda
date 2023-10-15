#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

class InverseDynamicsTorqueTrajectoryGenerator : public InverseDynamicsTorqueTrajectoryGeneratorBase
{
public:
    InverseDynamicsTorqueTrajectoryGenerator(std::ofstream &file,
                                             ClusterTreeModel<> &model_cl,
                                             const std::string &contact_point,
                                             const Trajectory &trajectory)
        : file_(file), contact_point_(contact_point), omega_(trajectory.omega_)
    {
        generateTorqueTrajectories(model_cl, trajectory);
    }

private:
    void perTimestepCallback(double t, ClusterTreeModel<> &model_cl) override
    {
        model_cl.forwardKinematicsIncludingContactPoints();
        const Vec3<double> cp_pos = model_cl.contactPoint(contact_point_).position_;
        file_ << omega_ << " " << t << " " << cp_pos.transpose() << std::endl;
    }

    std::ofstream &file_;
    const std::string contact_point_;
    const double omega_;
};

class OpenLoopSimulator : public OpenLoopSimulatorBase
{
public:
    OpenLoopSimulator(std::ofstream &file, const ClusterTreeModel<> &model,
                      const std::string &contact_point, const Trajectory &trajectory)
        : OpenLoopSimulatorBase(model, trajectory.at(0), trajectory.dt_),
          file_(file), contact_point_(contact_point), omega_(trajectory.omega_) {}

private:
    void
    preIntegrationCallback(double t, const ModelState<> &state, const DVec<double> &tau) override
    {
        // std::cout << "Simulator callback" << std::endl;
        model_.forwardKinematicsIncludingContactPoints();
        const Vec3<double> cp_pos = model_.contactPoint(contact_point_).position_;
        file_ << omega_ << " " << t << " " << cp_pos.transpose() << std::endl;
    }

    std::ofstream &file_;
    const std::string contact_point_;
    const double omega_;
};

template <typename RobotType>
void runOpenLoopTrajectoryBenchmark(std::ofstream &file, const std::string &cp_name,
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
        InverseDynamicsTorqueTrajectoryGenerator traj_generator(file, model, cp_name, trajectory);

        OpenLoopSimulator simulator(file, model, cp_name, trajectory);
        simulator.run(traj_generator.getTorqueTrajectory("exact"));
        simulator.run(traj_generator.getTorqueTrajectory("diag"));
        simulator.run(traj_generator.getTorqueTrajectory("none"));
    }
}

int main()
{
    // Trajectory Benchmark
    std::cout << "\n\n**Starting Trajectory Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyTT_";
    std::ofstream trajectory_file;
    trajectory_file.open(path_to_data + "Robots.csv");
    runOpenLoopTrajectoryBenchmark<MIT_Humanoid_Leg<>>(trajectory_file, "toe_contact", 6.0, 2e-3);

    // TODO(@MatthewChignoli: Tello joint requires spanning trajectory?
    // runOpenLoopTrajectoryBenchmark<TelloLeg>(trajectory_file, "toe_contact", 5.0, 1e-5);

    trajectory_file.close();

    return 0;
}
