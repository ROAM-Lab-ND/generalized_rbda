#include <iostream>
#include <casadi/casadi.hpp>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

using SX = casadi::SX;

template <typename Scalar>
using RobotType = MIT_Humanoid_Leg<Scalar>;
const int num_unique_links = 5;
const int num_unique_bodies = 10;

// TODO(@MatthewChignoli): Less duplicate code in this file!!!
void assignCasadiSymsToInertialParams(
    const SX &p, LinearInertialParams<SX> &inertial_params, int &idx)
{
    inertial_params.m = p(idx);
    casadi::copy(p(casadi::Slice(idx + 1, idx + 4)), inertial_params.h);
    casadi::copy(p(casadi::Slice(idx + 4, idx + 10)), inertial_params.I);
    idx += 10;
}

void assignCasadiSymsToLinkInertialParams(const SX &p, RobotType<SX> &robot, int &idx)
{
    assignCasadiSymsToInertialParams(p, robot._hipRzLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._hipRxLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._hipRyLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._kneeLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._ankleLinearInertialParams, idx);
}

void assignCasadiSymsToRotorInertialParams(const SX &p, RobotType<SX> &robot, int &idx)
{
    assignCasadiSymsToInertialParams(p, robot._hipRzRotorLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._hipRxRotorLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._hipRyRotorLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._kneeRotorLinearInertialParams, idx);
    assignCasadiSymsToInertialParams(p, robot._ankleRotorLinearInertialParams, idx);
}

casadi::Function getUnconstrainedRneaRegressorMatrixCasadi()
{
    // Robot info
    RobotType<SX> robot;

    // Create symbolic variables for the inertial parameters
    SX cs_p_sym = SX::sym("p", 10 * num_unique_bodies, 1);

    int idx = 0;
    assignCasadiSymsToLinkInertialParams(cs_p_sym, robot, idx);

    // Build symbolic model
    ClusterTreeModel<SX> model = robot.buildClusterTreeModel();
    grbda::ReflectedInertiaTreeModel<SX> approx_model(model, RotorInertiaApproximation::NONE);

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
    return casadi::Function("regressor_unconstrained", cs_inputs, cs_outputs);
}

casadi::Function getDiagonalApproxRneaRegressorMatrixCasadi()
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
    grbda::ReflectedInertiaTreeModel<SX> approx_model(model, RotorInertiaApproximation::DIAGONAL);

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
    return casadi::Function("regressor_diag_approx", cs_inputs, cs_outputs);
}

casadi::Function geExactRneaRegressorMatrixCasadi()
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
        DVec<SX> q_cluster = q_sym.segment(cluster->position_index_, cluster->num_positions_);
        DVec<SX> qd_cluster = qd_sym.segment(cluster->velocity_index_, cluster->num_velocities_);
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

std::vector<DVec<double>> generateTorqueTrajectory(int num_dof, const TrajectoryParameters &params)
{
    Trajectory trajectory(params);

    std::vector<DVec<double>> tau_traj;
    for (double t = 0; t <= params.duration; t += params.dt)
    {
        tau_traj.push_back(DVec<double>::Constant(num_dof, trajectory.at(t).p));
    }
    return tau_traj;
}

struct Sample
{
    ModelState<double> state;
    DVec<double> qdd;
    DVec<double> tau;
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

        Sample sample;
        sample.state = state;
        sample.qdd = model_.forwardDynamics(tau);
        sample.tau = tau;
        samples_.push_back(sample);

        t_last_collected_ = t;
    }

    std::vector<Sample> samples_;
    double t_last_collected_ = 0;
    const double collection_period_ = 0.2;
};

int main()
{
    // Create CasADi functions for the regressor matrices
    casadi::Function Y_unc = getUnconstrainedRneaRegressorMatrixCasadi();
    casadi::Function Y_diag = getDiagonalApproxRneaRegressorMatrixCasadi();
    casadi::Function Y_exact = geExactRneaRegressorMatrixCasadi();

    // Gather samples using the accurate dynamic model
    MIT_Humanoid<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    const int nv = model.getNumDegreesOfFreedom();

    // Generate the torque profiles that will be used to gather samples
    TrajectoryParameters traj_params;
    traj_params.dt = 5e-3;
    traj_params.duration = 2.0;

    std::vector<std::vector<DVec<double>>> tau_trajectories;
    for (double A = 1.; A <= 9.; A += 2.)
    {
        for (double omega = 0.1; omega <= 1.1; omega += 0.2)
        {
            for (double phi = 0.; phi <= 0.5; phi += 0.25)
            {
                traj_params.omega = omega;
                traj_params.phi = phi;
                traj_params.A = A;
                tau_trajectories.push_back(generateTorqueTrajectory(nv, traj_params));
            }
        }
    }

    // Simulate these trajectories to gather data
    TrajectoryPoint x0{0., 0., 0.};
    OpenLoopSimulator simulator(model, x0, traj_params.dt);
    std::vector<Sample> samples;
    for (const std::vector<DVec<double>> tau_traj : tau_trajectories)
    {
        simulator.run(tau_traj);
        for (const Sample &sample : simulator.getSamples())
        {
            samples.push_back(sample);
        }
    }
    std::cout << "Collected " << samples.size() << " samples" << std::endl;

    // For a bunch of different starting poses, simulate sinusoidal trajectories

    // // So yes, I think we need to first create a robot using symbolic variables for the inertia an stuff

    // // So now we need to set up the optimization variables such that we have 10 parameters per link

    // // Let's first just solve a simple QP to prove we can do it
    // // min 0.5*x'*Q*x + x'*q
    // // s.t. A*x <= b

    // casadi::Opti opti = casadi::Opti("conic"); // Optimization problem

    // casadi::Slice all;
    // // ---- decision variables ---------
    // casadi::MX x = opti.variable(2, 1);

    // // ---- objective          ---------
    // casadi::MX Q = casadi::MX::eye(2);
    // casadi::MX q = casadi::MX::ones(2, 1);
    // opti.minimize(0.5 * dot(x, mtimes(Q, x)) + dot(q, x));

    // // ---- constraints        ---------
    // opti.subject_to(x(0) >= 0.6);
    // opti.subject_to(x(1) >= 0);
    // opti.subject_to(x(0) + x(1) == 1);

    // // ---- solve              ---------
    // casadi::Dict qp_options;

    // opti.solver("ipqp", qp_options);    // set numerical backend
    // casadi::OptiSol sol = opti.solve(); // actual solve

    // std::cout << "x_opt = " << sol.value(x) << std::endl;

    return 0;
}
