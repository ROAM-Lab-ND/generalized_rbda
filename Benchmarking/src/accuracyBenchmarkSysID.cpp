#include <iostream>
#include <casadi/casadi.hpp>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

using SX = casadi::SX;
using MX = casadi::MX;

struct Sample
{
    double t;
    DVec<double> q;
    DVec<double> qd;
    DVec<double> qdd;
    DVec<double> tau;
};

class InverseDynamicsTorqueTrajectoryGenerator : public InverseDynamicsTorqueTrajectoryGeneratorBase
{
public:
    InverseDynamicsTorqueTrajectoryGenerator(ClusterTreeModel<> &model_cl,
                                             const Trajectory &trajectory)
    {
        generateTorqueTrajectories(model_cl, trajectory);
    }
};

class OpenLoopSimulator : public OpenLoopSimulatorBase
{
public:
    OpenLoopSimulator(const ClusterTreeModel<> &model, const TrajectoryPoint &x0, const double &dt)
        : OpenLoopSimulatorBase(model, x0, dt) {}

    const std::vector<Sample> &getSamples() const { return samples_; }

private:
    void
    preIntegrationCallback(double t, const ModelState<> &state, const DVec<double> &tau) override
    {
        if (t - t_last_collected_ < collection_period_)
        {
            return;
        }

        std::pair<DVec<double>, DVec<double>> q_and_qd = modelStateToVector(state);

        Sample sample;
        sample.t = t;
        sample.q = q_and_qd.first;
        sample.qd = q_and_qd.second;
        sample.qdd = model_.forwardDynamics(tau);
        sample.tau = tau;
        samples_.push_back(sample);

        t_last_collected_ = t;
    }

    void onExitCallback() override { t_last_collected_ = 0; }

    std::vector<Sample> samples_;
    double t_last_collected_ = 0;
    const double collection_period_ = 0.01;
};

class HumanoidLegSystemIdentificationBenchmark
{
public:
    template <typename Scalar>
    using RobotType = MIT_Humanoid_Leg<Scalar>;

    template <typename Scalar>
    using ApproxModel = grbda::ReflectedInertiaTreeModel<Scalar>;

    using TauTrajectory = std::vector<DVec<double>>;

    const int num_unique_links = 5;
    const int num_unique_bodies = 10;

    HumanoidLegSystemIdentificationBenchmark()
    {
        casadi::Function Y_unc = getUnconstrainedRneaRegressorMatrixCasadi();
        casadi::Function Y_diag = getDiagonalApproxRneaRegressorMatrixCasadi();
        casadi::Function Y_exact = geExactRneaRegressorMatrixCasadi();

        // Create accurate dynamic model for gathering samples
        RobotType<double> robot;
        ClusterTreeModel<double> model = robot.buildClusterTreeModel();
        const int nq = model.getNumPositions();
        const int nv = model.getNumDegreesOfFreedom();

        // Generate the torque profiles that will be used to gather samples
        std::vector<TauTrajectory> tau_trajectories;
        std::vector<TrajectoryPoint> starting_points;
        {
            TrajectoryParameters traj_params;
            traj_params.dt = dt_;
            traj_params.duration = 5.0;
            traj_params.phi = 0.0;

            for (double A : {0.1, 0.5, 1.0})
            {
                for (double omega : {0.1, 1.0, 2.0, 6.0})
                {
                    for (double phi : {0.0})
                    {
                        traj_params.omega = omega;
                        traj_params.phi = phi;
                        traj_params.A = A;
                        Trajectory trajectory(traj_params);
                        InverseDynamicsTorqueTrajectoryGenerator traj_generator(model,
                                                                                trajectory);
                        tau_trajectories.push_back(traj_generator.getTorqueTrajectory("exact"));
                        starting_points.push_back(trajectory.at(0));
                    }
                }
            }
        }
        std::cout << "Generated " << tau_trajectories.size()
                  << " torque trajectories" << std::endl;

        // Simulate these trajectories to gather data
        {
            std::ofstream sysID_data_file;
            sysID_data_file.open("../Benchmarking/data/HumanoidLeg_SysID_data.csv");
            for (size_t i = 0; i < tau_trajectories.size(); ++i)
            {
                OpenLoopSimulator simulator(model, starting_points[i], dt_);
                simulator.run(tau_trajectories[i]);
                for (const Sample &sample : simulator.getSamples())
                {
                    samples_.push_back(sample);
                    sysID_data_file << sample.t << "," << sample.q[3] << "," << sample.qd[3] << ","
                                    << sample.qdd[3] << "," << sample.tau[3] << std::endl;
                }
            }
            sysID_data_file.close();
            std::cout << "Collected " << samples_.size() << " samples" << std::endl;
        }

        // Solve the least squares problem to estimate the inertial parameters
        casadi::DM p_unc = estimateInertialParams(Y_unc);
        casadi::DM p_diag = estimateInertialParams(Y_diag);
        casadi::DM p_exact = estimateInertialParams(Y_exact);

        // Compare the performance of the estimated parameters on a new trajectory
        {
            std::ofstream results_file;
            results_file.open("../Benchmarking/data/AccuracySysID_HumanoidLeg.csv");

            // Generate the torque profiles we will test on
            TrajectoryParameters traj_params;
            traj_params.dt = dt_;
            traj_params.duration = 5.0;
            traj_params.omega = 1.0;
            traj_params.phi = 0.0;
            traj_params.A = 0.5;

            Trajectory trajectory(traj_params);
            InverseDynamicsTorqueTrajectoryGenerator traj_generator(model, trajectory);
            TauTrajectory tau_trajectory = traj_generator.getTorqueTrajectory("exact");

            // Simulate this trajectories to gather "ground truth" to compare against
            OpenLoopSimulator simulator(model, trajectory.at(0), dt_);
            simulator.run(tau_trajectory);

            // Record the different in predicted vs observed torque at the knee
            for (const Sample &sample : simulator.getSamples())
            {
                casadi::DM cs_q, cs_qd, cs_qdd, cs_tau;
                casadi::copy(sample.q, cs_q);
                casadi::copy(sample.qd, cs_qd);
                casadi::copy(sample.qdd, cs_qdd);
                casadi::copy(sample.tau, cs_tau);

                casadi::DMVector cs_inputs{cs_q, cs_qd, cs_qdd};
                casadi::DM tau_pred_unc = mtimes(Y_unc(cs_inputs)[0], p_unc);
                casadi::DM tau_pred_diag = mtimes(Y_diag(cs_inputs)[0], p_diag);
                casadi::DM tau_pred_exact = mtimes(Y_exact(cs_inputs)[0], p_exact);

                results_file << sample.t << "," << sample.tau[3] << "," << tau_pred_unc(3) << ","
                             << tau_pred_diag(3) << "," << tau_pred_exact(3) << "," << cs_q(3)
                             << std::endl;
            }
            results_file.close();
        }
    }

private:
    casadi::DM estimateInertialParams(casadi::Function &Y)
    {
        // TODO(@MatthewChignoli): Solve as a qp, but use a better QP solver. The ones that come with casadi seem pretty bad
        casadi::Opti opti = casadi::Opti(); // Optimization problem

        // ---- decision variables ---------
        const int num_params = Y.sparsity_out(0).size2();
        MX p = opti.variable(num_params, 1);

        // ---- objective          ---------
        casadi::MX cost = 0.0;
        for (const Sample &sample : samples_)
        {
            casadi::DM cs_q, cs_qd, cs_qdd, cs_tau;
            casadi::copy(sample.q, cs_q);
            casadi::copy(sample.qd, cs_qd);
            casadi::copy(sample.qdd, cs_qdd);
            casadi::copy(sample.tau, cs_tau);

            casadi::MX tau_pred = mtimes(Y(casadi::DMVector{cs_q, cs_qd, cs_qdd})[0], p);
            cost += dot(tau_pred - cs_tau, tau_pred - cs_tau);
        }
        opti.minimize(cost);

        // ---- solve              ---------
        opti.solver("knitro");              // set numerical backend
        casadi::OptiSol sol = opti.solve(); // actual solve
        return sol.value(p);
    }

    casadi::Function geExactRneaRegressorMatrixCasadi() const
    {
        // Robot info
        RobotType<SX> robot;

        // Create symbolic variables for the inertial parameters
        SX cs_p_sym = SX::sym("p", 10 * num_unique_bodies, 1);

        int idx = 0;
        assignCasadiSymsToLinkInertialParams(cs_p_sym, robot, idx);
        assignCasadiSymsToRotorInertialParams(cs_p_sym, robot, idx);

        // Build symbolic model
        ClusterTreeModel<SX> model = robot.buildClusterTreeModel();

        // Create symbolic variables for robot state
        SX cs_q_sym = SX::sym("q", model.getNumPositions(), 1);
        SX cs_qd_sym = SX::sym("qd", model.getNumDegreesOfFreedom(), 1);
        SX cs_qdd_sym = SX::sym("qdd", model.getNumDegreesOfFreedom(), 1);

        DVec<SX> q_sym(model.getNumPositions());
        casadi::copy(cs_q_sym, q_sym);

        DVec<SX> qd_sym(model.getNumDegreesOfFreedom());
        casadi::copy(cs_qd_sym, qd_sym);

        DVec<SX> qdd_sym(model.getNumDegreesOfFreedom());
        casadi::copy(cs_qdd_sym, qdd_sym);

        ModelState<SX> state;
        for (const ClusterTreeNodePtr<SX> &cluster : model.clusters())
        {
            DVec<SX> q_cluster = q_sym.segment(cluster->position_index_,
                                               cluster->num_positions_);
            DVec<SX> qd_cluster = qd_sym.segment(cluster->velocity_index_,
                                                 cluster->num_velocities_);
            JointState<SX> joint_state(JointCoordinate<SX>(q_cluster, false),
                                       JointCoordinate<SX>(qd_cluster, false));
            state.push_back(joint_state);
        }

        // Compute symbolic RNEA
        model.setState(state);
        DVec<SX> tau_sym = model.inverseDynamics(qdd_sym);

        // // Compute symbolic regressor matrix
        SX cs_tau_sym = SX(casadi::Sparsity::dense(model.getNumDegreesOfFreedom(), 1));
        casadi::copy(tau_sym, cs_tau_sym);
        SX cs_Y_sym = jacobian(cs_tau_sym, cs_p_sym);

        // Create casadi function
        std::vector<SX> cs_inputs{cs_q_sym, cs_qd_sym, cs_qdd_sym};
        std::vector<SX> cs_outputs{cs_Y_sym};
        return casadi::Function("regressor_exact", cs_inputs, cs_outputs);
    }

    casadi::Function getUnconstrainedRneaRegressorMatrixCasadi() const
    {
        RobotType<SX> robot;

        int idx = 0;
        SX cs_p_sym = SX::sym("p", 10 * num_unique_links, 1);
        assignCasadiSymsToLinkInertialParams(cs_p_sym, robot, idx);

        ClusterTreeModel<SX> model = robot.buildClusterTreeModel();
        ApproxModel<SX> approx_model(model, RotorInertiaApproximation::NONE);

        return getApproximateRneaRegressorMatrixCasadi(cs_p_sym, approx_model, "unconstrained");
    }

    casadi::Function getDiagonalApproxRneaRegressorMatrixCasadi() const
    {
        RobotType<SX> robot;

        int idx = 0;
        SX cs_p_sym = SX::sym("p", 10 * num_unique_bodies, 1);
        assignCasadiSymsToLinkInertialParams(cs_p_sym, robot, idx);
        assignCasadiSymsToRotorInertialParams(cs_p_sym, robot, idx);

        ClusterTreeModel<SX> model = robot.buildClusterTreeModel();
        ApproxModel<SX> approx_model(model, RotorInertiaApproximation::DIAGONAL);

        return getApproximateRneaRegressorMatrixCasadi(cs_p_sym, approx_model, "diag_approx");
    }

    casadi::Function getApproximateRneaRegressorMatrixCasadi(
        const SX &cs_p_sym, ApproxModel<SX> &approx_model, std::string name) const
    {
        // Create symbolic variables for robot state
        SX cs_q_sym = SX::sym("q", approx_model.getNumPositions(), 1);
        SX cs_qd_sym = SX::sym("qd", approx_model.getNumDegreesOfFreedom(), 1);
        SX cs_qdd_sym = SX::sym("qdd", approx_model.getNumDegreesOfFreedom(), 1);

        DVec<SX> q_sym(approx_model.getNumPositions());
        casadi::copy(cs_q_sym, q_sym);

        DVec<SX> qd_sym(approx_model.getNumDegreesOfFreedom());
        casadi::copy(cs_qd_sym, qd_sym);

        DVec<SX> qdd_sym(approx_model.getNumDegreesOfFreedom());
        casadi::copy(cs_qdd_sym, qdd_sym);

        // Compute symbolic RNEA
        approx_model.setIndependentStates(q_sym, qd_sym);
        DVec<SX> tau_sym = approx_model.inverseDynamics(qdd_sym);

        // // Compute symbolic regressor matrix
        SX cs_tau_sym = SX(casadi::Sparsity::dense(approx_model.getNumDegreesOfFreedom(), 1));
        casadi::copy(tau_sym, cs_tau_sym);
        SX cs_Y_sym = jacobian(cs_tau_sym, cs_p_sym);

        // Create casadi function
        std::vector<SX> cs_inputs{cs_q_sym, cs_qd_sym, cs_qdd_sym};
        std::vector<SX> cs_outputs{cs_Y_sym};
        return casadi::Function("regressor_" + name, cs_inputs, cs_outputs);
    }

    void assignCasadiSymsToLinkInertialParams(const SX &p, RobotType<SX> &robot, int &idx) const
    {
        assignCasadiSymsToInertialParams(p, robot._hipRzLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._hipRxLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._hipRyLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._kneeLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._ankleLinearInertialParams, idx);
    }

    void assignCasadiSymsToRotorInertialParams(const SX &p, RobotType<SX> &robot, int &idx) const
    {
        assignCasadiSymsToInertialParams(p, robot._hipRzRotorLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._hipRxRotorLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._hipRyRotorLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._kneeRotorLinearInertialParams, idx);
        assignCasadiSymsToInertialParams(p, robot._ankleRotorLinearInertialParams, idx);
    }

    void assignCasadiSymsToInertialParams(
        const SX &p, LinearInertialParams<SX> &inertial_params, int &idx) const
    {
        inertial_params.m = p(idx);
        casadi::copy(p(casadi::Slice(idx + 1, idx + 4)), inertial_params.h);
        casadi::copy(p(casadi::Slice(idx + 4, idx + 10)), inertial_params.I);
        idx += 10;
    }

    const double dt_ = 1e-4;
    std::vector<Sample> samples_;
};

int main()
{
    HumanoidLegSystemIdentificationBenchmark benchmark;
    return 0;
}
