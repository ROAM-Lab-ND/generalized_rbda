#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

class InverseDynamicsTorqueTrajectoryGenerator : public InverseDynamicsTorqueTrajectoryGeneratorBase
{
public:
    InverseDynamicsTorqueTrajectoryGenerator(ClusterTreeModel<> &model_cl,
                                             const Trajectory &trajectory,
                                             std::ofstream &torque_traj_file,
                                             std::ofstream &ee_traj_file,
                                             const std::string &contact_point)
        : torque_traj_file_(torque_traj_file), ee_traj_file_(ee_traj_file),
          contact_point_(contact_point), omega_(trajectory.omega_)
    {
        generateTorqueTrajectories(model_cl, trajectory);
    }

private:
    void perTimestepCallback(double t, ClusterTreeModel<> &model_cl) override
    {
        // End-effector trajectory
        model_cl.forwardKinematicsIncludingContactPoints();
        const Vec3<double> cp_pos = model_cl.contactPoint(contact_point_).position_;
        ee_traj_file_ << omega_ << " " << t << " " << cp_pos.transpose() << std::endl;

        // Torque trajectory
        torque_traj_file_ << omega_ << " " << t << " "
                          << tau_traj_map_["exact"].back().transpose() << " "
                          << tau_traj_map_["diag"].back().transpose() << " "
                          << tau_traj_map_["none"].back().transpose() << std::endl;
    }

    std::ofstream &torque_traj_file_;

    std::ofstream &ee_traj_file_;
    const std::string contact_point_;

    const double omega_;
};

class OpenLoopSimulator : public OpenLoopSimulatorBase
{
public:
    OpenLoopSimulator(const ClusterTreeModel<> &model, const Trajectory &trajectory,
                      std::ofstream &file, const std::string &contact_point)
        : OpenLoopSimulatorBase(model, trajectory.at(0), trajectory.dt_),
          ee_traj_file_(file), contact_point_(contact_point), omega_(trajectory.omega_) {}

private:
    void
    preIntegrationCallback(double t, const ModelState<> &state, const DVec<double> &tau) override
    {
        model_.forwardKinematicsIncludingContactPoints();
        const Vec3<double> cp_pos = model_.contactPoint(contact_point_).position_;
        ee_traj_file_ << omega_ << " " << t << " " << cp_pos.transpose() << std::endl;
    }

    std::ofstream &ee_traj_file_;
    const std::string contact_point_;
    const double omega_;
};

template <typename RobotType>
void runEndEffectorTrackingBenchmark(std::ofstream &tau_traj_file,
                                     std::ofstream &ee_traj_file, const std::string &cp_name,
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

    TrajectoryParameters trajectory_params;
    trajectory_params.A = 0.5;
    trajectory_params.phi = 0.;
    trajectory_params.dt = dt;
    trajectory_params.duration = duration;

    for (double omega : {0.25, 0.5, 0.75, 1.5})
    {
        trajectory_params.omega = omega * 2. * M_PI;
        Trajectory trajectory(trajectory_params);
        InverseDynamicsTorqueTrajectoryGenerator traj_generator(model, trajectory, tau_traj_file,
                                                                ee_traj_file, cp_name);

        OpenLoopSimulator simulator(model, trajectory, ee_traj_file, cp_name);
        simulator.run(traj_generator.getTorqueTrajectory("exact"));
        simulator.run(traj_generator.getTorqueTrajectory("diag"));
        simulator.run(traj_generator.getTorqueTrajectory("none"));
    }
}

int main()
{
    std::cout << "\n\n**Starting Trajectory Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyEET_";

    std::ofstream tau_trajectory_file;
    tau_trajectory_file.open(path_to_data + "HumanoidLeg_torque.csv");
    std::ofstream ee_trajectory_file;
    ee_trajectory_file.open(path_to_data + "HumanoidLeg_end_eff.csv");

    runEndEffectorTrackingBenchmark<MIT_Humanoid_Leg<>>(tau_trajectory_file, ee_trajectory_file,
                                                        "toe_contact", 5.0, 5e-4);

    tau_trajectory_file.close();
    ee_trajectory_file.close();

    // TODO(@MatthewChignoli: Tello joint requires spanning trajectory?
    // runOpenLoopTrajectoryBenchmark<TelloLeg>(trajectory_file, "toe_contact", 5.0, 1e-5);

    return 0;
}
