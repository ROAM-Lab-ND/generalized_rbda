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

int main()
{
    // Inverse Dynamics Benchmark
    std::cout << "\n\n**Starting Inverse Dynamics Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyID_";
    std::ofstream id_file;
    id_file.open(path_to_data + "Robots.csv");
    runInverseDynamicsBenchmark<TelloWithArms>(id_file, 50.);
    runInverseDynamicsBenchmark<MIT_Humanoid>(id_file, 50.);
    runInverseDynamicsBenchmark<MiniCheetah<>>(id_file, 30.);
    id_file.close();

    // Forward Dynamics Benchmark
    std::cout << "\n\n**Starting Forward Dynamics Accuracy Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/AccuracyFD_";
    std::ofstream fd_file;
    fd_file.open(path_to_data + "Robots.csv");
    runForwardDynamicsBenchmark<TelloWithArms>(fd_file, 50.);
    runForwardDynamicsBenchmark<MIT_Humanoid>(fd_file, 50.);
    runForwardDynamicsBenchmark<MiniCheetah<>>(fd_file, 20.);
    fd_file.close();

    // Inverse Operational Space Inertia Matrix Benchmark
    std::cout << "\n\n**Starting Inverse OSIM Accuracy Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/AccuracyIOSIM_";
    std::ofstream iosim_file;
    iosim_file.open(path_to_data + "Robots.csv");
    runInverseOperationalSpaceInertiaBenchmark<TelloWithArms>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<MIT_Humanoid>(iosim_file);
    runInverseOperationalSpaceInertiaBenchmark<MiniCheetah<>>(iosim_file);
    iosim_file.close();

    // Apply Test Force Benchmark
    std::cout << "\n\n**Starting Apply Test Force Accuracy Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/AccuracyATF_";
    std::ofstream atf_file;
    atf_file.open(path_to_data + "Robots.csv");
    runApplyTestForceBenchmark<TelloWithArms>(atf_file, "left-toe_contact", 500.);
    runApplyTestForceBenchmark<MIT_Humanoid>(atf_file, "left_toe_contact", 500.);
    runApplyTestForceBenchmark<MiniCheetah<>>(atf_file, "FL_foot_contact", 150.);
    atf_file.close();
}
