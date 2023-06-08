#include <iostream>
#include <fstream>

#include "DynamicsEngine/RigidBodyTreeModel.h"
#include "DynamicsEngine/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

template <class T>
void runBenchmark(std::ofstream &file)
{
    const int num_robot_samples = 400;
    const int num_state_samples = 25;

    double rf_error = 0.;
    double rf_diag_error = 0.;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        ClusterTreeModel cluster_model = robot.buildClusterTreeModel();
        ReflectedInertiaTreeModel reflected_inertia_model{cluster_model};

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_state_samples; j++)
        {

            // Set random state
            DVec<double> q = DVec<double>::Random(nq);
            DVec<double> qd = DVec<double>::Random(nv);
            cluster_model.initializeIndependentStates(q, qd);
            reflected_inertia_model.initializeIndependentStates(q, qd);

            // Forward Dynamics
            DVec<double> tau = DVec<double>::Random(nv);
            DVec<double> qdd_cluster = cluster_model.forwardDynamics(tau);
            DVec<double> qdd_approx = reflected_inertia_model.forwardDynamicsHandC(tau);
            DVec<double> qdd_diag_approx = reflected_inertia_model.forwardDynamics(tau);

            rf_error += (qdd_cluster - qdd_approx).norm();
            rf_diag_error += (qdd_cluster - qdd_diag_approx).norm();
        }
    }

    T robot;
    const size_t num_dof = robot.getNumDofs();
    const int num_samples = num_robot_samples * num_state_samples;
    file << num_dof << ","
         << rf_error / num_samples << ","
         << rf_diag_error / num_samples << std::endl;

    std::cout << "Finished benchmark for robot with " << num_dof << " dofs" << std::endl;
}

void runRevoluteWithRotorBenchmark()
{
    // Revolute With Rotors
    std::ofstream rev_file;
    rev_file.open("../Matlab_files/Results/RevoluteChainAccuracy.csv");

    runBenchmark<RevoluteChainWithRotor<2>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<4>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<6>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<8>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<12>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<16>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<20>>(rev_file);
    runBenchmark<RevoluteChainWithRotor<24>>(rev_file);

    rev_file.close();
}

void runRevolutePairWithRotorBenchmark()
{
    // Revolute Pair With Rotors
    std::ofstream rev_pair_file;
    rev_pair_file.open("../Matlab_files/Results/RevolutePairChainAccuracy.csv");

    runBenchmark<RevolutePairChainWithRotor<2>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<4>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<6>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<8>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<12>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<16>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<20>>(rev_pair_file);
    runBenchmark<RevolutePairChainWithRotor<24>>(rev_pair_file);

    rev_pair_file.close();
}

int main()
{
    runRevoluteWithRotorBenchmark();
    runRevolutePairWithRotorBenchmark();
    return 0;
}
