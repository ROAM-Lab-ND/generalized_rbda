#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"

#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::ClusterNodeVisitors;

template <class T>
void runBenchmark(std::ofstream &file, const std::string &id)
{
    const int num_robot_samples = 1000;
    const int num_state_samples = 10;
    RigidBodyTreeTimingStatistics timing_stats;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        const ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
        RigidBodyTreeModel lagrange_mult_model{cluster_model,
                                               FwdDynMethod::LagrangeMultiplierCustom};

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_state_samples; j++)
        {

            // Set random state
            ModelState model_state;
            DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
            DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
            for (const ClusterTreeModel::NodeTypeVariants &cluster : cluster_model.clusterVariants())
            {
                JointState joint_state = getJoint(cluster)->randomJointState();
                JointState spanning_joint_state = getJoint(cluster)->toSpanningTreeState(joint_state);

                spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                       spanning_joint_state.position);
                spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                       spanning_joint_state.velocity);

                model_state.push_back(joint_state);
            }

            lagrange_mult_model.initializeState(spanning_joint_pos, spanning_joint_vel);

            // Forward Dynamics
            DVec<double> tau = DVec<double>::Random(nv);
            DVec<double> qdd_lagrange = lagrange_mult_model.forwardDynamics(tau);

            timing_stats += lagrange_mult_model.getTimingStatistics();
        }
    }

    const int num_samples = num_robot_samples * num_state_samples;
    timing_stats /= num_samples;

    file << id << ","
         << timing_stats.ltl_factorization_time << ","
         << timing_stats.tau_prime_calc_time << ","
         << timing_stats.Y_and_z_calc_time << ","
         << timing_stats.A_and_b_time << ","
         << timing_stats.lambda_solve_time << ","
         << timing_stats.qdd_solve_time << std::endl;

    std::cout << "Finished benchmark for robot w/ id " << id << std::endl;
}

const std::string path_to_data = "../Benchmarking/data/LgMlt_";

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
