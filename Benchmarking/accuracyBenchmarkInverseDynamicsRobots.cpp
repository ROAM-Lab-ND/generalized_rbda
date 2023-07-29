#include <iostream>
#include <fstream>

#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

// TODO(@MatthewChignoli): Combine this with other accuracy benchmarks to avoid code duplication

template <typename RobotType>
void runBenchmark(std::ofstream &file)
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

            // Set random state
            ModelState model_state;
            DVec<double> independent_joint_pos = DVec<double>::Zero(0);
            DVec<double> independent_joint_vel = DVec<double>::Zero(0);
            DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
            DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

            bool nan_detected = false;
            for (const auto &cluster : model_cl.clusters())
            {
                JointState joint_state = cluster->joint_->randomJointState();
                if (joint_state.position.hasNaN())
                {
                    nan_detected = true;
                    break;
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
            if (nan_detected)
            {
                j--;
                continue;
            }

            model_cl.initializeState(model_state);
            model_rf.initializeIndependentStates(independent_joint_pos,
                                                 independent_joint_vel);
            model_rf_diag.initializeIndependentStates(independent_joint_pos,
                                                      independent_joint_vel);
            model_rf_none.initializeIndependentStates(independent_joint_pos,
                                                      independent_joint_vel);

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

int main()
{
    const std::string path_to_data = "../Benchmarking/data/AccuracyID_";

    std::cout << "\n\n**Starting Inverse Dynamics Timing Benchmark for Robots**" << std::endl;
    std::ofstream robots_file;
    robots_file.open(path_to_data + "Robots.csv");
    runBenchmark<Tello>(robots_file);
    runBenchmark<MIT_Humanoid>(robots_file);
    runBenchmark<MiniCheetah>(robots_file);
    robots_file.close();
}
