#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

class InverseDynamicsTorqueTrajectoryGenerator
{
public:
    typedef std::vector<DVec<double>> TauTrajectory;

    InverseDynamicsTorqueTrajectoryGenerator(std::ofstream &file,
                                             ClusterTreeModel<> &model_cl,
                                             const std::string &contact_point,
                                             const Trajectory &trajectory)
    {
        ReflectedInertiaTreeModel<> model_rf_diag(model_cl,
                                                  RotorInertiaApproximation::DIAGONAL);
        ReflectedInertiaTreeModel<> model_rf_none(model_cl,
                                                  RotorInertiaApproximation::NONE);

        const int nq = model_cl.getNumPositions();
        const int nv = model_cl.getNumDegreesOfFreedom();

        for (double t = 0; t <= trajectory.duration_; t += trajectory.dt_)
        {
            const TrajectoryPoint point = trajectory.at(t);
            DVec<double> q = DVec<double>::Constant(nq, point.p);
            DVec<double> qd = DVec<double>::Constant(nv, point.v);
            DVec<double> qdd = DVec<double>::Constant(nv, point.a);

            setState(model_cl, q, qd);
            setState(model_rf_diag, q, qd);
            setState(model_rf_none, q, qd);

            model_cl.forwardKinematicsIncludingContactPoints();
            const Vec3<double> cp_pos = model_cl.contactPoint(contact_point).position_;
            file << trajectory.omega_ << " " << t << " " << cp_pos.transpose() << std::endl;

            tau_traj_map_["exact"].push_back(model_cl.inverseDynamics(qdd));
            tau_traj_map_["diag"].push_back(model_rf_diag.inverseDynamics(qdd));
            tau_traj_map_["none"].push_back(model_rf_none.inverseDynamics(qdd));
        }
    }

    const TauTrajectory &getTorqueTrajectory(const std::string &model_name) const
    {
        return tau_traj_map_.at(model_name);
    }

protected:
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
            joint_state.position = q.segment(cluster->position_index_,
                                             cluster->num_positions_);
            joint_state.velocity = qd.segment(cluster->velocity_index_,
                                              cluster->num_velocities_);
            model_state.push_back(joint_state);
        }
        model_cl.setState(model_state);
    }

    std::map<std::string, TauTrajectory> tau_traj_map_;
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
