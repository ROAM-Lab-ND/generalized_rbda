#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

template <class T>
void runBenchmark(std::ofstream &file, const int ind_var, const bool include_forces)
{
    Timer timer;
    double t_cluster = 0.;
    double t_lagrange = 0.;
    double t_projection = 0.;
    double t_approx = 0.;

    const int num_robot_samples = 1000;
    const int num_state_samples = 10;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
        RigidBodyTreeModel lagrange_mult_model{cluster_model,
                                               ForwardDynamicsMethod::LagrangeMultiplier};
        RigidBodyTreeModel projection_model{cluster_model,
                                            ForwardDynamicsMethod::Projection};
        ReflectedInertiaTreeModel reflected_inertia_model{cluster_model};

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_state_samples; j++)
        {

            // Set random state
            DVec<double> q = DVec<double>::Random(nq);
            DVec<double> qd = DVec<double>::Random(nv);
            cluster_model.initializeIndependentStates(q, qd);
            lagrange_mult_model.initializeIndependentStates(q, qd);
            projection_model.initializeIndependentStates(q, qd);
            reflected_inertia_model.initializeIndependentStates(q, qd);

            // TODO(@MatthewChignoli): Seems like including forces does not slow it down? That's strange
            if (include_forces)
            {
                // Set random spatial forces on bodies
                std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
                for (const auto &body : cluster_model.bodies())
                    force_and_index_pairs.emplace_back(body.index_, SVec<double>::Random());
                cluster_model.initializeExternalForces(force_and_index_pairs);
                lagrange_mult_model.initializeExternalForces(force_and_index_pairs);
                projection_model.initializeExternalForces(force_and_index_pairs);
                reflected_inertia_model.initializeExternalForces(force_and_index_pairs);
            }

            // Forward Dynamics
            DVec<double> tau = DVec<double>::Random(nv);
            timer.start();
            DVec<double> qdd_cluster = cluster_model.forwardDynamics(tau);
            t_cluster += timer.getMs();

            timer.start();
            DVec<double> qdd_lagrange = lagrange_mult_model.forwardDynamics(tau);
            t_lagrange += timer.getMs();

            timer.start();
            DVec<double> qdd_projection = projection_model.forwardDynamics(tau);
            t_projection += timer.getMs();

            timer.start();
            DVec<double> qdd_approx = reflected_inertia_model.forwardDynamics(tau);
            t_approx += timer.getMs();
        }
    }

    T robot;
    const int num_samples = num_robot_samples * num_state_samples;
    file << ind_var << ","
         << t_cluster / num_samples << ","
         << t_lagrange / num_samples << "," << t_projection / num_samples << ","
         << t_approx / num_samples << std::endl;

    std::cout << "Finished benchmark for robot w/ independent variable " << ind_var << std::endl;
}

void runRevoluteWithRotorBenchmark(const bool include_forces)
{
    std::ofstream rev_file;
    rev_file.open("../Matlab_files/Results/RevoluteChain.csv");

    runBenchmark<RevoluteChainWithRotor<2>>(rev_file, 2, include_forces);
    runBenchmark<RevoluteChainWithRotor<4>>(rev_file, 4, include_forces);
    runBenchmark<RevoluteChainWithRotor<6>>(rev_file, 6, include_forces);
    runBenchmark<RevoluteChainWithRotor<8>>(rev_file, 8, include_forces);
    runBenchmark<RevoluteChainWithRotor<12>>(rev_file, 12, include_forces);
    runBenchmark<RevoluteChainWithRotor<16>>(rev_file, 16, include_forces);
    runBenchmark<RevoluteChainWithRotor<20>>(rev_file, 20, include_forces);
    runBenchmark<RevoluteChainWithRotor<24>>(rev_file, 24, include_forces);

    rev_file.close();
}

void runRevolutePairWithRotorBenchmark(const bool include_forces)
{
    std::ofstream rev_pair_file;
    rev_pair_file.open("../Matlab_files/Results/RevolutePairChain.csv");

    runBenchmark<RevolutePairChainWithRotor<2>>(rev_pair_file, 2, include_forces);
    runBenchmark<RevolutePairChainWithRotor<4>>(rev_pair_file, 4, include_forces);
    runBenchmark<RevolutePairChainWithRotor<6>>(rev_pair_file, 6, include_forces);
    runBenchmark<RevolutePairChainWithRotor<8>>(rev_pair_file, 8, include_forces);
    runBenchmark<RevolutePairChainWithRotor<12>>(rev_pair_file, 12, include_forces);
    runBenchmark<RevolutePairChainWithRotor<16>>(rev_pair_file, 16, include_forces);
    runBenchmark<RevolutePairChainWithRotor<20>>(rev_pair_file, 20, include_forces);
    runBenchmark<RevolutePairChainWithRotor<24>>(rev_pair_file, 24, include_forces);

    rev_pair_file.close();
}

void runLoopSizeBenchmark(const bool include_forces)
{
    // This benchmark finds the trend in computation time when the number of clusters is held constant, but the size of every cluster is varied

    std::ofstream results_file;
    results_file.open("../Matlab_files/Results/RevoluteMultiRotorChain.csv");

    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 1>>(results_file, 1, include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 2>>(results_file, 2, include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 3>>(results_file, 3, include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 4>>(results_file, 4, include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 5>>(results_file, 5, include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 6>>(results_file, 6, include_forces);
    runBenchmark<RevoluteChainMultipleRotorsPerLink<4, 7>>(results_file, 7, include_forces);

    results_file.close();
}

void runConstraintDimensionBenchmark(const bool include_forces)
{
    // This benchmark finds the trend in computation time when the number of independent coordinates is held constant, but the number of constraints is varied

    std::ofstream results_file;
    results_file.open("../Matlab_files/Results/RevoluteWithAndWithoutRotorChain.csv");

    runBenchmark<RevoluteChainWithAndWithoutRotor<0, 8>>(results_file, 0, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<1, 7>>(results_file, 1, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<2, 6>>(results_file, 2, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<3, 5>>(results_file, 3, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<4, 4>>(results_file, 4, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<5, 3>>(results_file, 5, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<6, 2>>(results_file, 6, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<7, 1>>(results_file, 7, include_forces);
    runBenchmark<RevoluteChainWithAndWithoutRotor<8, 0>>(results_file, 8, include_forces);

    results_file.close();
}

int main()
{
    const bool include_forces = false;
    runRevoluteWithRotorBenchmark(include_forces);
    runRevolutePairWithRotorBenchmark(include_forces);
    runLoopSizeBenchmark(include_forces);
    runConstraintDimensionBenchmark(include_forces);
    return 0;
}
