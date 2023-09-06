#include <iostream>
#include <fstream>

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"

#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

template <class T>
void runBenchmark(std::ofstream &file)
{
    const int num_robot_samples = 400;
    const int num_state_samples = 25;

    double rf_lambda_inv_error = 0.;
    double rf_diag_lambda_inv_error = 0.;

    double rf_dstate_error = 0.;
    double rf_diag_dstate_error = 0.;

    for (int i = 0; i < num_robot_samples; i++)
    {
        T robot = T(true);
        ClusterTreeModel cl_model = robot.buildClusterTreeModel();
        ReflectedInertiaTreeModel rf_model{cl_model, RotorInertiaApproximation::BLOCK_DIAGONAL};
        ReflectedInertiaTreeModel rf_diag_model{cl_model, RotorInertiaApproximation::DIAGONAL};

        const int nq = cl_model.getNumPositions();
        const int nv = cl_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_state_samples; j++)
        {

            // Set random state
            ModelState model_state;
            DVec<double> independent_joint_pos = DVec<double>::Zero(0);
            DVec<double> independent_joint_vel = DVec<double>::Zero(0);
            for (const auto &cluster : cl_model.clusters())
            {
                JointState joint_state = cluster->joint_->randomJointState();
                if (joint_state.position.isSpanning() || joint_state.velocity.isSpanning())
                    throw std::runtime_error("Initializing reflected inertia model requires all independent coordinates");

                independent_joint_pos = appendEigenVector(independent_joint_pos,
                                                          joint_state.position);
                independent_joint_vel = appendEigenVector(independent_joint_vel,
                                                          joint_state.velocity);

                model_state.push_back(joint_state);
            }
            cl_model.initializeState(model_state);
            rf_model.initializeIndependentStates(independent_joint_pos, independent_joint_vel);
            rf_diag_model.initializeIndependentStates(independent_joint_pos, independent_joint_vel);

            // Apply Test Force
            DVec<double> dstate;
            DVec<double> dstate_approx;
            DVec<double> dstate_diag;

            const Vec3<double> test_force = 50. * Vec3<double>::Random();
            const std::string cp_name = cl_model.contactPoints().back().name_;

            const double lambda_inv = cl_model.applyTestForce(cp_name, test_force, dstate);
            const double lambda_inv_approx = rf_model.applyTestForce(cp_name, test_force,
                                                                     dstate_approx);
            const double lambda_inv_diag = rf_diag_model.applyTestForce(cp_name, test_force,
                                                                        dstate_diag);

            rf_lambda_inv_error += std::fabs(lambda_inv - lambda_inv_approx); 
            rf_diag_lambda_inv_error += std::fabs(lambda_inv - lambda_inv_diag);

            rf_dstate_error += (dstate - dstate_approx).norm();
            rf_diag_dstate_error += (dstate - dstate_diag).norm();
        }
    }

    T robot;
    const size_t num_dof = robot.getNumDofs();
    const int num_samples = num_robot_samples * num_state_samples;
    file << num_dof << ","
            << rf_lambda_inv_error / num_samples << ","
            << rf_diag_lambda_inv_error / num_samples << ","
            << rf_dstate_error / num_samples << ","
            << rf_diag_dstate_error / num_samples << std::endl;

    std::cout << "Finished benchmark for robot with " << num_dof << " dofs" << std::endl;
}

const std::string path_to_data = "../Benchmarking/data/AccuracyATF_";

void runRevoluteWithRotorBenchmark()
{
    // Revolute With Rotors
    std::ofstream rev_file;
    rev_file.open(path_to_data + "RevoluteChain.csv");

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
    rev_pair_file.open(path_to_data + "RevolutePairChain.csv");

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
