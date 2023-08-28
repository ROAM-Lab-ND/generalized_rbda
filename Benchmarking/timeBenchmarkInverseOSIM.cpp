#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

template <class T>
void runBenchmark(std::ofstream &file, const std::string& id)
{
    Timer timer;
    double t_cluster = 0.;
    double t_projection = 0.;
    double t_approx = 0.;

    const int num_robot_samples = 300;
    const int num_state_samples = 10;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
        RigidBodyTreeModel projection_model{cluster_model,
                                            FwdDynMethod::Projection};
        ReflectedInertiaTreeModel reflected_inertia_model{cluster_model,
                                                          RotorInertiaApproximation::DIAGONAL};

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_state_samples; j++)
        {

            // Set random state
            ModelState model_state;
            DVec<double> independent_joint_pos = DVec<double>::Zero(0);
            DVec<double> independent_joint_vel = DVec<double>::Zero(0);
            DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
            DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
            for (const auto &cluster : cluster_model.clusters())
            {
                JointState joint_state = cluster->joint_->randomJointState();
                if (joint_state.position.isSpanning() || joint_state.velocity.isSpanning())
                    throw std::runtime_error("Initializing reflected inertia model requires all independent coordinates");
                    
                independent_joint_pos = appendEigenVector(independent_joint_pos,
                                                          joint_state.position);
                independent_joint_vel = appendEigenVector(independent_joint_vel,
                                                          joint_state.velocity);

                JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);
                spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                       spanning_joint_state.position);
                spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                       spanning_joint_state.velocity);

                model_state.push_back(joint_state);
            }

            cluster_model.initializeState(model_state);
            projection_model.initializeState(spanning_joint_pos, spanning_joint_vel);
            reflected_inertia_model.initializeIndependentStates(independent_joint_pos,
                                                                independent_joint_vel);

            // Inverse Operational Space Inertia Matrix
            timer.start();
            cluster_model.inverseOperationalSpaceInertiaMatrix();
            t_cluster += timer.getMs();

            timer.start();
            projection_model.inverseOperationalSpaceInertiaMatrix();
            t_projection += timer.getMs();

            timer.start();
            reflected_inertia_model.inverseOperationalSpaceInertiaMatrix();
            t_approx += timer.getMs();
        }
    }

    const int num_samples = num_robot_samples * num_state_samples;
    file << id << ","
         << t_cluster / num_samples << ","
         << t_projection / num_samples << ","
         << t_approx / num_samples << std::endl;

    std::cout << "Finished benchmark for robot w/ id " << id << std::endl;
}

const std::string path_to_data = "../Benchmarking/data/TimingIOSIM_";

void runRevoluteWithRotorBenchmark()
{
    std::cout << "** Revolute With Rotor Benchmark **" << std::endl;
 
    std::ofstream rev_file;
    rev_file.open(path_to_data + "RevoluteChain.csv");

    runBenchmark<RevoluteChainWithRotor<2>>(rev_file, "2");
    runBenchmark<RevoluteChainWithRotor<4>>(rev_file, "4");
    runBenchmark<RevoluteChainWithRotor<6>>(rev_file, "6");
    runBenchmark<RevoluteChainWithRotor<8>>(rev_file, "8");
    runBenchmark<RevoluteChainWithRotor<12>>(rev_file, "12");
    runBenchmark<RevoluteChainWithRotor<16>>(rev_file, "16");
    runBenchmark<RevoluteChainWithRotor<20>>(rev_file, "20");
    runBenchmark<RevoluteChainWithRotor<24>>(rev_file, "24");

    rev_file.close();
}

void runRevolutePairWithRotorBenchmark()
{
    std::cout << "** Revolute Pair With Rotor Benchmark **" << std::endl;

    std::ofstream rev_pair_file;
    rev_pair_file.open(path_to_data + "RevolutePairChain.csv");

    runBenchmark<RevolutePairChainWithRotor<2>>(rev_pair_file, "2");
    runBenchmark<RevolutePairChainWithRotor<4>>(rev_pair_file, "4");
    runBenchmark<RevolutePairChainWithRotor<6>>(rev_pair_file, "6");
    runBenchmark<RevolutePairChainWithRotor<8>>(rev_pair_file, "8");
    runBenchmark<RevolutePairChainWithRotor<12>>(rev_pair_file, "12");
    runBenchmark<RevolutePairChainWithRotor<16>>(rev_pair_file, "16");
    runBenchmark<RevolutePairChainWithRotor<20>>(rev_pair_file, "20");
    runBenchmark<RevolutePairChainWithRotor<24>>(rev_pair_file, "24");

    rev_pair_file.close();
}

int main()
{
    runRevoluteWithRotorBenchmark();
    runRevolutePairWithRotorBenchmark();
    return 0;
}
