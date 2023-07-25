#include <iostream>
#include <fstream>

#include "Dynamics/ClusterTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

template <class T>
void runBenchmark(std::ofstream &file, const std::string &id)
{
    const int num_robot_samples = 1000;
    const int num_state_samples = 10;
    ClusterTreeTimingStatistics timing_stats;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        ClusterTreeModel cluster_model = robot.buildClusterTreeModel();

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_state_samples; j++)
        {

            // Set random state
            ModelState model_state;
            for (const ClusterTreeNode &cluster : cluster_model.clusters())
            {
                model_state.push_back(cluster.joint_->randomJointState());
            }

            cluster_model.initializeState(model_state);

            // Forward Dynamics
            DVec<double> tau = DVec<double>::Random(nv);
            cluster_model.forwardDynamics(tau);

            timing_stats += cluster_model.getTimingStatistics();
        }
    }

    const int num_samples = num_robot_samples * num_state_samples;
    timing_stats /= num_samples;

    file << id << ","
         << timing_stats.forward_kinematics_time << ","
         << timing_stats.update_articulated_bodies_time << ","
         << timing_stats.forward_pass1_time << ","
         << timing_stats.external_force_time << ","
         << timing_stats.backward_pass_time << ","
         << timing_stats.forward_pass2_time << ","
         << timing_stats.invert_xform_spatial_inertia_time << ","
         << timing_stats.update_and_solve_D_time << ","
         << timing_stats.reset_IA_time << std::endl;

    std::cout
        << "Finished benchmark for robot w/ id " << id << std::endl;
}

const std::string path_to_data = "../Benchmarking/data/CL_";

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
