#include <iostream>
#include <fstream>

#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

bool setRandomStates(ClusterTreeModel &cf_model,
                     ReflectedInertiaTreeModel &model_rf,
                     ReflectedInertiaTreeModel &model_rf_diag,
                     ReflectedInertiaTreeModel &model_rf_none)
{
    ModelState model_state;
    DVec<double> independent_joint_pos = DVec<double>::Zero(0);
    DVec<double> independent_joint_vel = DVec<double>::Zero(0);
    DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
    DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

    for (const auto &cluster : cf_model.clusters())
    {
        JointState joint_state = cluster->joint_->randomJointState();
        if (joint_state.position.hasNaN())
        {
            return true;
        }

        JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

        DVec<double> independent_joint_pos_i;
        DVec<double> independent_joint_vel_i;
        if (cluster->joint_->type() == GeneralizedJointTypes::TelloHipDifferential ||
            cluster->joint_->type() == GeneralizedJointTypes::TelloKneeAnkleDifferential)
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

    cf_model.initializeState(model_state);
    model_rf.initializeIndependentStates(independent_joint_pos,
                                         independent_joint_vel);
    model_rf_diag.initializeIndependentStates(independent_joint_pos,
                                              independent_joint_vel);
    model_rf_none.initializeIndependentStates(independent_joint_pos,
                                              independent_joint_vel);

    return false;
}

template <typename RobotType>
void runInverseDynamicsBenchmark(std::ofstream &file)
{
    RobotType robot;
    ClusterTreeModel model_cl = robot.buildClusterTreeModel();
    ReflectedInertiaTreeModel model_rf(model_cl, RotorInertiaApproximation::BLOCK_DIAGONAL);
    ReflectedInertiaTreeModel model_rf_diag(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreeModel model_rf_none(model_cl, RotorInertiaApproximation::NONE);

    const int nv = model_cl.getNumDegreesOfFreedom();

    const double max_torque = 55.;
    const double alpha_rate = 0.1;
    const int num_samples = 1000;
    for (double alpha = 0.; alpha < 1.; alpha += alpha_rate)
    {
        double id_error = 0.;
        double id_diag_error = 0.;
        double id_none_error = 0.;

        for (int j = 0; j < num_samples; j++)
        {
            bool nan_detected = setRandomStates(model_cl, model_rf, model_rf_diag, model_rf_none);
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
            const DVec<double> tau_approx = model_rf.inverseDynamics(qdd);
            const DVec<double> tau_diag_approx = model_rf_diag.inverseDynamics(qdd);
            const DVec<double> tau_none_approx = model_rf_none.inverseDynamics(qdd);

            id_error += (tau_cluster - tau_approx).norm();
            id_diag_error += (tau_cluster - tau_diag_approx).norm();
            id_none_error += (tau_cluster - tau_none_approx).norm();
        }

        file << alpha << ","
             << id_none_error / num_samples << ","
             << id_diag_error / num_samples << ","
             << id_error / num_samples << std::endl;
    }
}

template <typename RobotType>
void runForwardDynamicsBenchmark(std::ofstream &file)
{
    RobotType robot;
    ClusterTreeModel model_cl = robot.buildClusterTreeModel();
    ReflectedInertiaTreeModel model_rf(model_cl, RotorInertiaApproximation::BLOCK_DIAGONAL);
    ReflectedInertiaTreeModel model_rf_diag(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreeModel model_rf_none(model_cl, RotorInertiaApproximation::NONE);

    const int nv = model_cl.getNumDegreesOfFreedom();

    const double max_torque = 55.;
    const double alpha_rate = 0.1;
    const int num_samples = 1000;
    for (double alpha = 0.; alpha < 1.; alpha += alpha_rate)
    {
        double fd_diag_error = 0.;
        double fd_none_error = 0.;

        for (int j = 0; j < num_samples; j++)
        {
            bool nan_detected = setRandomStates(model_cl, model_rf, model_rf_diag, model_rf_none);
            if (nan_detected)
            {
                j--;
                continue;
            }

            // Forward Dynamics
            DVec<double> tau = alpha * max_torque * DVec<double>::Random(nv);
            tau.head<6>().setZero();
            const DVec<double> qdd_cluster = model_cl.forwardDynamics(tau);
            const DVec<double> qdd_diag_approx = model_rf_diag.forwardDynamics(tau);
            const DVec<double> qdd_none_approx = model_rf_none.forwardDynamics(tau);

            fd_diag_error += (qdd_cluster - qdd_diag_approx).norm();
            fd_none_error += (qdd_cluster - qdd_none_approx).norm();
        }

        file << alpha << ","
             << fd_none_error / num_samples << ","
             << fd_diag_error / num_samples << std::endl;
    }
}

template <typename RobotType>
void runApplyTestForceBenchmark(std::ofstream &file, const std::string contact_point)
{
    RobotType robot;
    ClusterTreeModel model_cl = robot.buildClusterTreeModel();
    ReflectedInertiaTreeModel model_rf(model_cl, RotorInertiaApproximation::BLOCK_DIAGONAL);
    ReflectedInertiaTreeModel model_rf_diag(model_cl, RotorInertiaApproximation::DIAGONAL);
    ReflectedInertiaTreeModel model_rf_none(model_cl, RotorInertiaApproximation::NONE);

    const int nv = model_cl.getNumDegreesOfFreedom();

    const double max_force = 200.;
    const double alpha_rate = 0.1;
    const int num_samples = 1000;
    for (double alpha = 0.; alpha < 1.; alpha += alpha_rate)
    {
        double dstate_diag_error = 0.;
        double dstate_none_error = 0.;

        for (int j = 0; j < num_samples; j++)
        {
            bool nan_detected = setRandomStates(model_cl, model_rf, model_rf_diag, model_rf_none);
            if (nan_detected)
            {
                j--;
                continue;
            }

            // Apply Test Force
            Vec3<double> force = alpha * max_force * Vec3<double>::Random();
            DVec<double> dstate_cluster;
            model_cl.applyLocalFrameTestForceAtContactPoint(force, contact_point,
                                                            dstate_cluster);
            DVec<double> dstate_diag_approx;
            model_rf_diag.applyLocalFrameTestForceAtContactPoint(force, contact_point,
                                                                 dstate_diag_approx);
            DVec<double> dstate_none_approx;
            model_rf_none.applyLocalFrameTestForceAtContactPoint(force, contact_point,
                                                                 dstate_none_approx);

            dstate_diag_error += (dstate_cluster - dstate_diag_approx).norm();
            dstate_none_error += (dstate_cluster - dstate_none_approx).norm();
        }

        file << alpha << ","
             << dstate_none_error / num_samples << ","
             << dstate_diag_error / num_samples << std::endl;
    }
}

int main()
{
    // Inverse Dynamics Benchmark
    std::cout << "\n\n**Starting Inverse Dynamics Accuracy Benchmark for Robots**" << std::endl;
    std::string path_to_data = "../Benchmarking/data/AccuracyID_";
    std::ofstream id_file;
    id_file.open(path_to_data + "Robots.csv");
    runInverseDynamicsBenchmark<Tello>(id_file);
    runInverseDynamicsBenchmark<MIT_Humanoid>(id_file);
    runInverseDynamicsBenchmark<MiniCheetah>(id_file);
    id_file.close();

    // Forward Dynamics Benchmark
    std::cout << "\n\n**Starting Forward Dynamics Accuracy Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/AccuracyFD_";
    std::ofstream fd_file;
    fd_file.open(path_to_data + "Robots.csv");
    runForwardDynamicsBenchmark<Tello>(fd_file);
    runForwardDynamicsBenchmark<MIT_Humanoid>(fd_file);
    runForwardDynamicsBenchmark<MiniCheetah>(fd_file);
    fd_file.close();

    // Apply Test Force Benchmark
    std::cout << "\n\n**Starting Apply Test Force Accuracy Benchmark for Robots**" << std::endl;
    path_to_data = "../Benchmarking/data/AccuracyATF_";
    std::ofstream atf_file;
    atf_file.open(path_to_data + "Robots.csv");
    runApplyTestForceBenchmark<Tello>(atf_file, "left-toe_contact");
    runApplyTestForceBenchmark<MIT_Humanoid>(atf_file, "left_toe_contact");
    runApplyTestForceBenchmark<MiniCheetah>(atf_file, "FL_foot_contact");
    atf_file.close();
}
