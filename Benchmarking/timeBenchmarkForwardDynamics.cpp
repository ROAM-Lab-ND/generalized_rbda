#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Timer.h"

using namespace grbda;

template <class T>
void runBenchmark(std::ofstream &file, const std::string& id, const bool include_forces)
{
    Timer timer;
    double t_cluster = 0.;
    double t_lg_custom = 0.;
    double t_lg_eigen = 0.;
    double t_projection = 0.;
    double t_approx = 0.;

    const int num_robot_samples = 300;
    const int num_state_samples = 10;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
        RigidBodyTreeModel lg_custom_mult_model{cluster_model,
                                                FwdDynMethod::LagrangeMultiplierCustom};
        RigidBodyTreeModel lg_eigen_mult_model{cluster_model,
                                               FwdDynMethod::LagrangeMultiplierEigen};
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

            cluster_model.setState(model_state);
            lg_custom_mult_model.setState(spanning_joint_pos, spanning_joint_vel);
            lg_eigen_mult_model.setState(spanning_joint_pos, spanning_joint_vel);
            projection_model.setState(spanning_joint_pos, spanning_joint_vel);
            reflected_inertia_model.setIndependentStates(independent_joint_pos,
                                                                independent_joint_vel);

            // TODO(@MatthewChignoli): Seems like including forces does not slow it down? That's strange
            if (include_forces)
            {
                // Set random spatial forces on bodies
                std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
                for (const auto &body : cluster_model.bodies())
                    force_and_index_pairs.emplace_back(body.index_, SVec<double>::Random());
                cluster_model.setExternalForces(force_and_index_pairs);
                lg_custom_mult_model.setExternalForces(force_and_index_pairs);
                lg_eigen_mult_model.setExternalForces(force_and_index_pairs);
                projection_model.setExternalForces(force_and_index_pairs);
                reflected_inertia_model.setExternalForces(force_and_index_pairs);
            }

            // Forward Dynamics
            DVec<double> tau = DVec<double>::Random(nv);
            timer.start();
            cluster_model.forwardDynamics(tau);
            t_cluster += timer.getMs();

            timer.start();
            lg_custom_mult_model.forwardDynamics(tau);
            t_lg_custom += timer.getMs();

            timer.start();
            lg_eigen_mult_model.forwardDynamics(tau);
            t_lg_eigen += timer.getMs();

            timer.start();
            projection_model.forwardDynamics(tau);
            t_projection += timer.getMs();

            timer.start();
            reflected_inertia_model.forwardDynamics(tau);
            t_approx += timer.getMs();
        }
    }

    const int num_samples = num_robot_samples * num_state_samples;
    file << id << ","
         << t_cluster / num_samples << ","
         << t_lg_custom / num_samples << ","
         << t_lg_eigen / num_samples << ","
         << t_projection / num_samples << ","
         << t_approx / num_samples << std::endl;

    std::cout << "Finished benchmark for robot w/ id " << id << std::endl;
}

const std::string path_to_data = "../Benchmarking/data/Timing_";

void runRevoluteWithRotorBenchmark(const bool include_forces)
{
    std::cout << "** Revolute With Rotor Benchmark **" << std::endl;
 
    std::ofstream rev_file;
    rev_file.open(path_to_data + "RevoluteChain.csv");

    runBenchmark<RevoluteChainWithRotor<2>>(rev_file, "2", include_forces);
    runBenchmark<RevoluteChainWithRotor<4>>(rev_file, "4", include_forces);
    runBenchmark<RevoluteChainWithRotor<6>>(rev_file, "6", include_forces);
    runBenchmark<RevoluteChainWithRotor<8>>(rev_file, "8", include_forces);
    runBenchmark<RevoluteChainWithRotor<12>>(rev_file, "12", include_forces);
    runBenchmark<RevoluteChainWithRotor<16>>(rev_file, "16", include_forces);
    runBenchmark<RevoluteChainWithRotor<20>>(rev_file, "20", include_forces);
    runBenchmark<RevoluteChainWithRotor<24>>(rev_file, "24", include_forces);

    rev_file.close();
}

void runRevolutePairWithRotorBenchmark(const bool include_forces)
{
    std::cout << "** Revolute Pair With Rotor Benchmark **" << std::endl;

    std::ofstream rev_pair_file;
    rev_pair_file.open(path_to_data + "RevolutePairChain.csv");

    runBenchmark<RevolutePairChainWithRotor<2>>(rev_pair_file, "2", include_forces);
    runBenchmark<RevolutePairChainWithRotor<4>>(rev_pair_file, "4", include_forces);
    runBenchmark<RevolutePairChainWithRotor<6>>(rev_pair_file, "6", include_forces);
    runBenchmark<RevolutePairChainWithRotor<8>>(rev_pair_file, "8", include_forces);
    runBenchmark<RevolutePairChainWithRotor<12>>(rev_pair_file, "12", include_forces);
    runBenchmark<RevolutePairChainWithRotor<16>>(rev_pair_file, "16", include_forces);
    runBenchmark<RevolutePairChainWithRotor<20>>(rev_pair_file, "20", include_forces);
    runBenchmark<RevolutePairChainWithRotor<24>>(rev_pair_file, "24", include_forces);

    rev_pair_file.close();
}

void runLoopSizeBenchmark(const bool include_forces)
{
    // This benchmark finds the trend in computation time when the number of clusters is held constant, but the size of every cluster is varied

    std::cout << "** Revolute Multi-Rotor Chain Benchmark **" << std::endl;

    std::ofstream results_file;
    results_file.open(path_to_data + "RevoluteMultiRotorChain.csv");

    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 1>>(results_file, "4,1", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 2>>(results_file, "4,2", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 3>>(results_file, "4,3", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 4>>(results_file, "4,4", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 5>>(results_file, "4,5", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 6>>(results_file, "4,6", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 7>>(results_file, "4,7", include_forces);

    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 1>>(results_file, "8,1", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 2>>(results_file, "8,2", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 3>>(results_file, "8,3", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 4>>(results_file, "8,4", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 5>>(results_file, "8,5", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 6>>(results_file, "8,6", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<8, 7>>(results_file, "8,7", include_forces);

    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 1>>(results_file, "16,1", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 2>>(results_file, "16,2", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 3>>(results_file, "16,3", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 4>>(results_file, "16,4", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 5>>(results_file, "16,5", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 6>>(results_file, "16,6", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<16, 7>>(results_file, "16,7", include_forces);

    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 1>>(results_file, "24,1", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 2>>(results_file, "24,2", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 3>>(results_file, "24,3", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 4>>(results_file, "24,4", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 5>>(results_file, "24,5", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 6>>(results_file, "24,6", include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<24, 7>>(results_file, "24,7", include_forces);

    results_file.close();
}

void runConstraintDimensionBenchmark(const bool include_forces)
{
    // This benchmark finds the trend in computation time when the number of independent coordinates is held constant, but the number of constraints is varied

    std::cout << "** Revolute With And Without Rotor Chain Benchmark **" << std::endl;

    std::ofstream results_file;
    results_file.open(path_to_data + "RevoluteWithAndWithoutRotorChain.csv");

    runBenchmark<RevoluteChainWithAndWithoutRotor<0, 8>>(results_file, "0,8", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<1, 7>>(results_file, "1,7", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<2, 6>>(results_file, "2,6", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<3, 5>>(results_file, "3,5", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<4, 4>>(results_file, "4,4", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<5, 3>>(results_file, "5,3", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<6, 2>>(results_file, "6,2", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<7, 1>>(results_file, "7,1", include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<8, 0>>(results_file, "8,0", include_forces);

    results_file.close();
}

int main()
{
    const bool include_forces = false;
    runRevoluteWithRotorBenchmark(include_forces);
    runRevolutePairWithRotorBenchmark(include_forces);
    // runLoopSizeBenchmark(include_forces);
    // runConstraintDimensionBenchmark(include_forces);
    return 0;
}
