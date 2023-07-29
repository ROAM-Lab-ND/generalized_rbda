#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

template <typename RobotType>
void runBenchmark(std::ofstream &file)
{
    Timer timer;
    double t_cluster = 0.;
    double t_lg_custom = 0.;
    double t_lg_eigen = 0.;
    double t_projection = 0.;
    double t_reflected_inertia = 0.;

    const int num_state_samples = 1000;

    RobotType robot;
    ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
    RigidBodyTreeModel lg_custom_mult_model{cluster_model,
                                            FwdDynMethod::LagrangeMultiplierCustom};
    RigidBodyTreeModel lg_eigen_mult_model{cluster_model,
                                           FwdDynMethod::LagrangeMultiplierEigen};
    RigidBodyTreeModel projection_model{cluster_model,
                                        FwdDynMethod::Projection};
    ReflectedInertiaTreeModel reflected_inertia_model(cluster_model,
                                                      RotorInertiaApproximation::DIAGONAL);

    const int nq = cluster_model.getNumPositions();
    const int nv = cluster_model.getNumDegreesOfFreedom();

    for (int i = 0; i < num_state_samples; i++)
    {
        // Set random state
        ModelState model_state;
        DVec<double> independent_joint_pos = DVec<double>::Zero(0);
        DVec<double> independent_joint_vel = DVec<double>::Zero(0);
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

        bool nan_detected = false;
        for (const auto &cluster : cluster_model.clusters())
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
            i--;
            continue;
        }

        cluster_model.initializeState(model_state);
        lg_custom_mult_model.initializeState(spanning_joint_pos, spanning_joint_vel);
        lg_eigen_mult_model.initializeState(spanning_joint_pos, spanning_joint_vel);
        projection_model.initializeState(spanning_joint_pos, spanning_joint_vel);
        reflected_inertia_model.initializeIndependentStates(independent_joint_pos,
                                                            independent_joint_vel);

        // Forward Dynamics
        DVec<double> tau = DVec<double>::Random(nv);
        timer.start();
        DVec<double> ydd_cluster = cluster_model.forwardDynamics(tau);
        t_cluster += timer.getMs();

        timer.start();
        DVec<double> qdd_lg_custom = lg_custom_mult_model.forwardDynamics(tau);
        t_lg_custom += timer.getMs();

        timer.start();
        DVec<double> qdd_lg_eigen = lg_eigen_mult_model.forwardDynamics(tau);
        t_lg_eigen += timer.getMs();

        timer.start();
        DVec<double> qdd_proj = projection_model.forwardDynamics(tau);
        t_projection += timer.getMs();

        timer.start();
        reflected_inertia_model.forwardDynamics(tau);
        t_reflected_inertia += timer.getMs();
    }

    file << t_cluster / num_state_samples << ","
         << t_lg_custom / num_state_samples << ","
         << t_lg_eigen / num_state_samples << ","
         << t_projection / num_state_samples << ","
         << t_reflected_inertia / num_state_samples << std::endl;

    std::cout << "Finished benchmark for robot" << std::endl;
}

int main()
{
    const std::string path_to_data = "../Benchmarking/data/TimingFD_";

    std::cout << "\n\n**Starting Forward Dynamics Timing Benchmark for Robots**" << std::endl;
    std::ofstream robots_file;
    robots_file.open(path_to_data + "Robots.csv");
    runBenchmark<Tello>(robots_file);
    runBenchmark<MIT_Humanoid>(robots_file);
    runBenchmark<MiniCheetah>(robots_file);
    robots_file.close();
}
