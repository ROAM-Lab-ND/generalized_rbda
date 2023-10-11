#include <iostream>
#include <fstream>

#include "BenchmarkingHelpers.hpp"
#include "Robots/RobotTypes.h"
#include "Utils/Timer.h"

using namespace grbda;
using namespace grbda::BenchmarkHelpers;

template <typename RobotType>
void runInverseDynamicsBenchmark(std::ofstream &file, const double max_torque)
{
    RobotType robot;
    ClusterTreeModel<> model_cl = robot.buildClusterTreeModel();

    ReflectedInertiaTreePtr model_rf = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::BLOCK_DIAGONAL);
    ReflectedInertiaTreePtr model_rf_diag = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreePtr model_rf_none = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::NONE);
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{model_rf, model_rf_diag, model_rf_none};

    const int nv = model_cl.getNumDegreesOfFreedom();

    const double alpha_rate = 0.1;
    const int num_samples_per_alpha = 2000.;
    for (double alpha = 0.; alpha <= 1.; alpha += alpha_rate)
    {
        double id_error = 0.;
        double id_diag_error = 0.;
        double id_none_error = 0.;

        for (int j = 0; j < num_samples_per_alpha; j++)
        {
            bool nan_detected = setRandomStates(model_cl, {}, ref_inertia_models);
            if (nan_detected)
            {
                j--;
                continue;
            }

            // Sample qdd
            DVec<double> tau = alpha * max_torque * DVec<double>::Random(nv);
            tau.head<6>().setZero();
            const DVec<double> qdd = model_cl.forwardDynamics(tau);

            // Inverse Dynamics
            const DVec<double> tau_cluster = model_cl.inverseDynamics(qdd);
            const DVec<double> tau_approx = model_rf->inverseDynamics(qdd);
            const DVec<double> tau_diag_approx = model_rf_diag->inverseDynamics(qdd);
            const DVec<double> tau_none_approx = model_rf_none->inverseDynamics(qdd);

            id_error += (tau_cluster - tau_approx).norm();
            id_diag_error += (tau_cluster - tau_diag_approx).norm();
            id_none_error += (tau_cluster - tau_none_approx).norm();
        }

        file << alpha << ","
             << id_none_error / num_samples_per_alpha << ","
             << id_diag_error / num_samples_per_alpha << ","
             << id_error / num_samples_per_alpha << std::endl;
    }
}

template <typename RobotType>
void runForwardDynamicsBenchmark(std::ofstream &file, const double max_torque)
{
    RobotType robot;
    ClusterTreeModel<> model_cl = robot.buildClusterTreeModel();

    ReflectedInertiaTreePtr model_rf_diag = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreePtr model_rf_none = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::NONE);
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{model_rf_diag, model_rf_none};

    const int nv = model_cl.getNumDegreesOfFreedom();

    const double alpha_rate = 0.1;
    const int num_samples_per_alpha = 2000;
    for (double alpha = 0.; alpha <= 1.; alpha += alpha_rate)
    {
        double fd_diag_error = 0.;
        double fd_none_error = 0.;

        for (int j = 0; j < num_samples_per_alpha; j++)
        {
            bool nan_detected = setRandomStates(model_cl, {}, ref_inertia_models);
            if (nan_detected)
            {
                j--;
                continue;
            }

            // Forward Dynamics
            DVec<double> tau = alpha * max_torque * DVec<double>::Random(nv);
            tau.head<6>().setZero();
            const DVec<double> qdd_cluster = model_cl.forwardDynamics(tau);
            const DVec<double> qdd_diag_approx = model_rf_diag->forwardDynamics(tau);
            const DVec<double> qdd_none_approx = model_rf_none->forwardDynamics(tau);

            fd_diag_error += (qdd_cluster - qdd_diag_approx).norm();
            fd_none_error += (qdd_cluster - qdd_none_approx).norm();
        }

        file << alpha << ","
             << fd_none_error / num_samples_per_alpha << ","
             << fd_diag_error / num_samples_per_alpha << std::endl;
    }
}

template <typename RobotType>
void runInverseOperationalSpaceInertiaBenchmark(std::ofstream &file)
{
    RobotType robot;
    ClusterTreeModel<> model_cl = robot.buildClusterTreeModel();

    ReflectedInertiaTreePtr model_rf_diag = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreePtr model_rf_none = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::NONE);
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{model_rf_diag, model_rf_none};

    double lambda_inv_diag_rel_error = 0.;
    double lambda_inv_none_rel_error = 0.;

    const int num_samples = 2000;
    for (int j = 0; j < num_samples; j++)
    {
        bool nan_detected = setRandomStates(model_cl, {}, ref_inertia_models);
        if (nan_detected)
        {
            j--;
            continue;
        }

        const DMat<double> lambda_inv = model_cl.inverseOperationalSpaceInertiaMatrix();
        const DMat<double> lambda_inv_diag = model_rf_diag->inverseOperationalSpaceInertiaMatrix();
        const DMat<double> lambda_inv_none = model_rf_none->inverseOperationalSpaceInertiaMatrix();

        lambda_inv_diag_rel_error += (lambda_inv - lambda_inv_diag).norm() / lambda_inv.norm();
        lambda_inv_none_rel_error += (lambda_inv - lambda_inv_none).norm() / lambda_inv.norm();
    }

    file << lambda_inv_none_rel_error / num_samples << ","
         << lambda_inv_diag_rel_error / num_samples << std::endl;
}

template <typename RobotType>
void runApplyTestForceBenchmark(std::ofstream &file, const std::string contact_point,
                                const double max_force)
{
    RobotType robot;
    ClusterTreeModel<> model_cl = robot.buildClusterTreeModel();

    ReflectedInertiaTreePtr model_rf_diag = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreePtr model_rf_none = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::NONE);
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{model_rf_diag, model_rf_none};

    const int nv = model_cl.getNumDegreesOfFreedom();

    const double alpha_rate = 0.1;
    const double offset = 0.01;
    const int num_samples_per_alpha = 2000;
    for (double alpha = offset; alpha <= (1. + offset); alpha += alpha_rate)
    {
        double dstate_diag_error = 0.;
        double dstate_none_error = 0.;

        for (int j = 0; j < num_samples_per_alpha; j++)
        {
            bool nan_detected = setRandomStates(model_cl, {}, ref_inertia_models);
            if (nan_detected)
            {
                j--;
                continue;
            }

            // Apply Test Force
            Vec3<double> force = alpha * max_force * Vec3<double>::Random();
            DVec<double> dstate_cluster;
            model_cl.applyTestForce(contact_point, force, dstate_cluster);
            DVec<double> dstate_diag_approx;
            model_rf_diag->applyTestForce(contact_point, force, dstate_diag_approx);
            DVec<double> dstate_none_approx;
            model_rf_none->applyTestForce(contact_point, force, dstate_none_approx);

            dstate_diag_error += (dstate_cluster - dstate_diag_approx).norm();
            dstate_none_error += (dstate_cluster - dstate_none_approx).norm();
        }

        file << alpha << ","
             << dstate_none_error / num_samples_per_alpha << ","
             << dstate_diag_error / num_samples_per_alpha << std::endl;
    }
}

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
    const double omega = 2. * M_PI * 1.0;
    const double phi = 0.;

    TrajectoryPoint point;
    point.p = A * sin(omega * t + phi);
    point.v = A * omega * cos(omega * t + phi);
    point.a = -A * omega * omega * sin(omega * t + phi);
    return point;
}

ModelState<> eulerIntegrateModelState(ModelState<> model_state, const ClusterTreeModel<> &model,
                                      const DVec<double> &qdd, const double dt)
{
    for (size_t i = 0; i < model_state.size(); i++)
    {
        JointState<double> &joint_state = model_state[i];
        const auto &cluster = model.cluster(i);

        // Euler integration
        joint_state.position += dt * joint_state.velocity;
        joint_state.velocity += dt * qdd.segment(cluster->velocity_index_,
                                                 cluster->num_velocities_);
    }

    return model_state;
}

template <typename RobotType>
void runOpenLoopTrajectoryBenchmark(std::ofstream &file, const std::string &contact_point)
{
    // TODO(@MatthewChignoli): Write a description of this benchmark

    using TauTrajectory = std::vector<DVec<double>>;

    RobotType robot;
    ClusterTreeModel<> model_cl = robot.buildClusterTreeModel();

    ReflectedInertiaTreePtr model_rf_diag = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreePtr model_rf_none = std::make_shared<ReflectedInertiaTreeModel<>>(model_cl, RotorInertiaApproximation::NONE);
    std::vector<ReflectedInertiaTreePtr> ref_inertia_models{model_rf_diag, model_rf_none};

    const int nq = model_cl.getNumPositions();
    const int nv = model_cl.getNumDegreesOfFreedom();

    // Generate the torque trajectory using the approximate model
    double T = 5.0;
    const double dt = 5e-5;
    TauTrajectory tau_diag_trajectory;
    TauTrajectory tau_none_trajectory;
    TauTrajectory tau_exact_trajectory;
    for (double t = 0; t <= T; t += dt)
    {
        // Set State
        const TrajectoryPoint point = trajectory(t);
        DVec<double> q = DVec<double>::Constant(nq, point.p);
        DVec<double> qd = DVec<double>::Constant(nv, point.v);
        DVec<double> qdd = DVec<double>::Constant(nv, point.a);

        model_rf_diag->setIndependentStates(q, qd);
        model_rf_none->setIndependentStates(q, qd);

        ModelState<> model_state;
        for (const auto &cluster : model_cl.clusters())
        {
            JointState<> joint_state;
            joint_state.position = DVec<double>::Constant(cluster->num_positions_, point.p);
            joint_state.velocity = DVec<double>::Constant(cluster->num_velocities_, point.v);
            model_state.push_back(joint_state);
        }
        model_cl.setState(model_state);

        // Compute Desired Contact Point Position
        model_rf_diag->forwardKinematicsIncludingContactPoints();
        const ContactPoint<double> &cp = model_rf_diag->contactPoint(contact_point);
        file << t << " " << cp.position_.transpose() << std::endl;

        // Compute torque needed to track the trajectory via inverse dynamics
        tau_diag_trajectory.push_back(model_rf_diag->inverseDynamics(qdd));
        tau_none_trajectory.push_back(model_rf_none->inverseDynamics(qdd));
        tau_exact_trajectory.push_back(model_cl.inverseDynamics(qdd));
    }

    // Simulate the exact model forward in time using the various torque trajectories
    for (const TauTrajectory &tau_trajectory : {tau_exact_trajectory,
                                                tau_diag_trajectory,
                                                tau_none_trajectory})
    {
        ModelState<> model_state;
        for (const auto &cluster : model_cl.clusters())
        {
            const TrajectoryPoint point = trajectory(0);
            JointState<> joint_state;
            joint_state.position = DVec<double>::Constant(cluster->num_positions_, point.p);
            joint_state.velocity = DVec<double>::Constant(cluster->num_velocities_, point.v);
            model_state.push_back(joint_state);
        }
        model_cl.setState(model_state);

        double t = 0;
        for (const DVec<double> &tau : tau_trajectory)
        {
            // Compute the realized Contact Point Position
            model_cl.forwardKinematicsIncludingContactPoints();
            const ContactPoint<double> &cp = model_cl.contactPoint(contact_point);
            file << t << " " << cp.position_.transpose() << std::endl;

            // TODO(@MatthewChignoli): This won't work with floating base joint
            // Simulate forward in time using the torque from the approximate model

            // Euler
            // DVec<double> qdd = model_cl.forwardDynamics(tau);
            // for (size_t i = 0; i < model_state.size(); i++)
            // {
            //     JointState<double> &joint_state = model_state[i];
            //     const auto &cluster = model_cl.cluster(i);

            //     // Euler integration
            //     joint_state.position += dt * joint_state.velocity;
            //     joint_state.velocity += dt * qdd.segment(cluster->velocity_index_,
            //                                              cluster->num_velocities_);

            // }

            // RK4
            {
                // Compute k1
                DVec<double> k1 = model_cl.forwardDynamics(tau);

                // Compute k2
                ModelState<> model_state_k2 = eulerIntegrateModelState(model_state, model_cl,
                                                                       k1, dt / 2.);
                model_cl.setState(model_state_k2);
                DVec<double> k2 = model_cl.forwardDynamics(tau);

                // Compute k3
                ModelState<> model_state_k3 = eulerIntegrateModelState(model_state, model_cl,
                                                                       k2, dt / 2.);
                model_cl.setState(model_state_k3);
                DVec<double> k3 = model_cl.forwardDynamics(tau);

                // Compute k4
                ModelState<> model_state_k4 = eulerIntegrateModelState(model_state, model_cl,
                                                                       k3, dt);
                model_cl.setState(model_state_k4);
                DVec<double> k4 = model_cl.forwardDynamics(tau);

                // Compute qdd
                DVec<double> qdd = (k1 + 2. * k2 + 2. * k3 + k4) / 6.;

                for (size_t i = 0; i < model_state.size(); i++)
                {
                    const auto &cluster = model_cl.cluster(i);

                    JointState<double> &joint_state = model_state[i];
                    const JointState<double> &joint_state_k2 = model_state_k2[i];
                    const JointState<double> &joint_state_k3 = model_state_k3[i];
                    const JointState<double> &joint_state_k4 = model_state_k4[i];

                    DVec<double> qd = (joint_state.velocity + 2. * joint_state_k2.velocity +
                                       2. * joint_state_k3.velocity + joint_state_k4.velocity) /
                                      6.;

                    // Euler integration
                    joint_state.position += dt * qd;
                    joint_state.velocity += dt * qdd.segment(cluster->velocity_index_,
                                                             cluster->num_velocities_);

                }
            }

            model_cl.setState(model_state);

            t += dt;
        }
    }
}

int main()
{
    // // Inverse Dynamics Benchmark
    // std::cout << "\n\n**Starting Inverse Dynamics Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyID_";
    // std::ofstream id_file;
    // id_file.open(path_to_data + "Robots.csv");
    // runInverseDynamicsBenchmark<TelloWithArms>(id_file, 50.);
    // runInverseDynamicsBenchmark<MIT_Humanoid<>>(id_file, 50.);
    // runInverseDynamicsBenchmark<MiniCheetah<>>(id_file, 30.);
    // id_file.close();

    // // Forward Dynamics Benchmark
    // std::cout << "\n\n**Starting Forward Dynamics Accuracy Benchmark for Robots**" << std::endl;
    // path_to_data = "../Benchmarking/data/AccuracyFD_";
    // std::ofstream fd_file;
    // fd_file.open(path_to_data + "Robots.csv");
    // runForwardDynamicsBenchmark<TelloWithArms>(fd_file, 50.);
    // runForwardDynamicsBenchmark<MIT_Humanoid<>>(fd_file, 50.);
    // runForwardDynamicsBenchmark<MiniCheetah<>>(fd_file, 20.);
    // fd_file.close();

    // // Inverse Operational Space Inertia Matrix Benchmark
    // std::cout << "\n\n**Starting Inverse OSIM Accuracy Benchmark for Robots**" << std::endl;
    // path_to_data = "../Benchmarking/data/AccuracyIOSIM_";
    // std::ofstream iosim_file;
    // iosim_file.open(path_to_data + "Robots.csv");
    // runInverseOperationalSpaceInertiaBenchmark<TelloWithArms>(iosim_file);
    // runInverseOperationalSpaceInertiaBenchmark<MIT_Humanoid<>>(iosim_file);
    // runInverseOperationalSpaceInertiaBenchmark<MiniCheetah<>>(iosim_file);
    // iosim_file.close();

    // // Apply Test Force Benchmark
    // std::cout << "\n\n**Starting Apply Test Force Accuracy Benchmark for Robots**" << std::endl;
    // path_to_data = "../Benchmarking/data/AccuracyATF_";
    // std::ofstream atf_file;
    // atf_file.open(path_to_data + "Robots.csv");
    // runApplyTestForceBenchmark<TelloWithArms>(atf_file, "left-toe_contact", 500.);
    // runApplyTestForceBenchmark<MIT_Humanoid<>>(atf_file, "left_toe_contact", 500.);
    // runApplyTestForceBenchmark<MiniCheetah<>>(atf_file, "FL_foot_contact", 150.);
    // atf_file.close();

    // Trajectory Benchmark
    std::cout << "\n\n**Starting Trajectory Accuracy Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/AccuracyTT_";
    std::ofstream trajectory_file;
    trajectory_file.open(path_to_data + "Robots.csv");
    runOpenLoopTrajectoryBenchmark<RevoluteChainWithRotor<6>>(trajectory_file, "cp-5");
    trajectory_file.close();
}
