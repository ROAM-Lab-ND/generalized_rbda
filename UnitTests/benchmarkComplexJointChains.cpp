#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

struct ChainResult {
    std::string chain_type;
    int num_links;
    int dof;
    double avg_time_us;
    double max_error_dq;
    double max_error_dqdot;
};

template <typename ChainType>
ChainResult testChain(const std::string& chain_name, int num_links) {
    try {
        // Build the cluster tree model from the robot/chain specification
        ChainType chain;
        ClusterTreeModel<double> model = chain.buildClusterTreeModel();
        int nDOF = model.getNumDegreesOfFreedom();
        
        if (nDOF == 0) {
            throw std::runtime_error("Model has zero DOF");
        }
        
        // Set random state - build ModelState by iterating over clusters
        ModelState<double> model_state;
        for (const auto& cluster : model.clusters()) {
            model_state.push_back(cluster->joint_->randomJointState());
        }
        model.setState(model_state);
        auto [q, qd] = model.getState();
        DVec<double> ydd = DVec<double>::Random(nDOF);

        // Compute analytical derivatives (warmup)
        for (int i = 0; i < 100; ++i) {
            auto [dtau_dq_analytical, dtau_dqdot_analytical] = 
                model.firstOrderInverseDynamicsDerivatives(ydd);
        }

        // Timed runs
        const int num_iterations = 1000;
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_iterations; ++i) {
            auto [dtau_dq_analytical, dtau_dqdot_analytical] = 
                model.firstOrderInverseDynamicsDerivatives(ydd);
        }
        auto end = std::chrono::high_resolution_clock::now();
        
        double total_time_us = std::chrono::duration<double, std::micro>(end - start).count();
        double avg_time_us = total_time_us / num_iterations;

        // Compute analytical derivatives for error checking
        auto [dtau_dq_analytical, dtau_dqdot_analytical] = 
            model.firstOrderInverseDynamicsDerivatives(ydd);

        // Compute numerical derivatives using centered finite differences
        const double h = 1e-7;
        DMat<double> dtau_dq_numerical(nDOF, nDOF);
        DMat<double> dtau_dqdot_numerical(nDOF, nDOF);

        // Finite differences for ∂τ/∂q
        for (int i = 0; i < nDOF; ++i) {
            DVec<double> q_plus = q;
            DVec<double> q_minus = q;
            q_plus(i) += h;
            q_minus(i) -= h;
            
            ModelState<double> state_plus;
            int idx = 0;
            for (const auto& cluster : model.clusters()) {
                JointState<double> js;
                js.position = q_plus.segment(idx, cluster->num_positions_);
                js.velocity = qd.segment(idx, cluster->num_velocities_);
                state_plus.push_back(js);
                idx += cluster->num_velocities_;
            }
            model.setState(state_plus);
            DVec<double> tau_plus = model.inverseDynamics(ydd);
            
            ModelState<double> state_minus;
            idx = 0;
            for (const auto& cluster : model.clusters()) {
                JointState<double> js;
                js.position = q_minus.segment(idx, cluster->num_positions_);
                js.velocity = qd.segment(idx, cluster->num_velocities_);
                state_minus.push_back(js);
                idx += cluster->num_velocities_;
            }
            model.setState(state_minus);
            DVec<double> tau_minus = model.inverseDynamics(ydd);
            
            dtau_dq_numerical.col(i) = (tau_plus - tau_minus) / (2.0 * h);
        }

        // Finite differences for ∂τ/∂qd
        for (int i = 0; i < nDOF; ++i) {
            DVec<double> qd_plus = qd;
            DVec<double> qd_minus = qd;
            qd_plus(i) += h;
            qd_minus(i) -= h;
            
            ModelState<double> state_plus;
            int idx = 0;
            for (const auto& cluster : model.clusters()) {
                JointState<double> js;
                js.position = q.segment(idx, cluster->num_positions_);
                js.velocity = qd_plus.segment(idx, cluster->num_velocities_);
                state_plus.push_back(js);
                idx += cluster->num_velocities_;
            }
            model.setState(state_plus);
            DVec<double> tau_plus = model.inverseDynamics(ydd);
            
            ModelState<double> state_minus;
            idx = 0;
            for (const auto& cluster : model.clusters()) {
                JointState<double> js;
                js.position = q.segment(idx, cluster->num_positions_);
                js.velocity = qd_minus.segment(idx, cluster->num_velocities_);
                state_minus.push_back(js);
                idx += cluster->num_velocities_;
            }
            model.setState(state_minus);
            DVec<double> tau_minus = model.inverseDynamics(ydd);
            
            dtau_dqdot_numerical.col(i) = (tau_plus - tau_minus) / (2.0 * h);
        }

        // Restore original state
        model.setState(model_state);

        // Compute errors
        DMat<double> error_dq = (dtau_dq_analytical - dtau_dq_numerical).cwiseAbs();
        DMat<double> error_dqdot = (dtau_dqdot_analytical - dtau_dqdot_numerical).cwiseAbs();

        ChainResult result;
        result.chain_type = chain_name;
        result.num_links = num_links;
        result.dof = nDOF;
        result.avg_time_us = avg_time_us;
        result.max_error_dq = error_dq.maxCoeff();
        result.max_error_dqdot = error_dqdot.maxCoeff();

        return result;
    } catch (const std::exception& e) {
        std::cerr << "Error testing " << chain_name << ": " << e.what() << "\n";
        ChainResult result;
        result.chain_type = chain_name + " (ERROR)";
        result.num_links = num_links;
        result.dof = -1;
        result.avg_time_us = 0;
        result.max_error_dq = 0;
        result.max_error_dqdot = 0;
        return result;
    }
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Complex Joint Chain Scaling Benchmark\n";
    std::cout << "Comparing RevolutePairChainWithRotor and RevoluteTripleChainWithRotor\n";
    std::cout << "===========================================================================\n\n";

    // Test 1: RevolutePairChainWithRotor
    std::cout << "Test 1: RevolutePairChainWithRotor - Coupled Pairs with Transmission\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(25) << "Chain Type"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<ChainResult> pair_rotor_results;
    
    auto pair2 = testChain<RevolutePairChainWithRotor<2, double>>("RevPairWithRotor_2", 2);
    pair_rotor_results.push_back(pair2);
    std::cout << std::left << std::setw(25) << pair2.chain_type
              << std::setw(6) << pair2.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << pair2.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << pair2.max_error_dq
              << std::setw(14) << pair2.max_error_dqdot
              << "\n";
    
    auto pair4 = testChain<RevolutePairChainWithRotor<4, double>>("RevPairWithRotor_4", 4);
    pair_rotor_results.push_back(pair4);
    std::cout << std::left << std::setw(25) << pair4.chain_type
              << std::setw(6) << pair4.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << pair4.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << pair4.max_error_dq
              << std::setw(14) << pair4.max_error_dqdot
              << "\n";

    std::cout << "---------------------------------------------------------------------------\n\n";

    // Test 2: RevoluteTripleChainWithRotor
    std::cout << "Test 2: RevoluteTripleChainWithRotor - Triple Coupled with Transmission\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(25) << "Chain Type"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<ChainResult> triple_rotor_results;
    
    auto triple3 = testChain<RevoluteTripleChainWithRotor<3, double>>("RevTripleWithRotor_3", 3);
    triple_rotor_results.push_back(triple3);
    std::cout << std::left << std::setw(25) << triple3.chain_type
              << std::setw(6) << triple3.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << triple3.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << triple3.max_error_dq
              << std::setw(14) << triple3.max_error_dqdot
              << "\n";
    
    auto triple6 = testChain<RevoluteTripleChainWithRotor<6, double>>("RevTripleWithRotor_6", 6);
    triple_rotor_results.push_back(triple6);
    std::cout << std::left << std::setw(25) << triple6.chain_type
              << std::setw(6) << triple6.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << triple6.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << triple6.max_error_dq
              << std::setw(14) << triple6.max_error_dqdot
              << "\n";

    std::cout << "---------------------------------------------------------------------------\n\n";

    // Test 3: Baseline comparison with simple RevoluteChainWithRotor
    std::cout << "Test 3: Baseline - RevoluteChainWithRotor (Simple Revolutes)\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(25) << "Chain Type"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<ChainResult> simple_results;
    
    auto simple2 = testChain<RevoluteChainWithRotor<2, double>>("RevoluteWithRotor_2", 2);
    simple_results.push_back(simple2);
    std::cout << std::left << std::setw(25) << simple2.chain_type
              << std::setw(6) << simple2.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << simple2.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << simple2.max_error_dq
              << std::setw(14) << simple2.max_error_dqdot
              << "\n";
    
    auto simple3 = testChain<RevoluteChainWithRotor<3, double>>("RevoluteWithRotor_3", 3);
    simple_results.push_back(simple3);
    std::cout << std::left << std::setw(25) << simple3.chain_type
              << std::setw(6) << simple3.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << simple3.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << simple3.max_error_dq
              << std::setw(14) << simple3.max_error_dqdot
              << "\n";
    
    auto simple4 = testChain<RevoluteChainWithRotor<4, double>>("RevoluteWithRotor_4", 4);
    simple_results.push_back(simple4);
    std::cout << std::left << std::setw(25) << simple4.chain_type
              << std::setw(6) << simple4.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << simple4.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << simple4.max_error_dq
              << std::setw(14) << simple4.max_error_dqdot
              << "\n";
    
    auto simple6 = testChain<RevoluteChainWithRotor<6, double>>("RevoluteWithRotor_6", 6);
    simple_results.push_back(simple6);
    std::cout << std::left << std::setw(25) << simple6.chain_type
              << std::setw(6) << simple6.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << simple6.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << simple6.max_error_dq
              << std::setw(14) << simple6.max_error_dqdot
              << "\n";

    std::cout << "---------------------------------------------------------------------------\n\n";

    // Comparison Analysis
    std::cout << "\n===========================================================================\n";
    std::cout << "Performance Comparison Analysis\n";
    std::cout << "===========================================================================\n\n";

    // 2-DOF / 2-link systems
    std::cout << "Two-DOF Systems (2 links):\n";
    std::cout << "  Simple Revolute:       " << std::fixed << std::setprecision(2) 
              << simple2.avg_time_us << " us\n";
    std::cout << "  RevolutePairWithRotor: " << std::fixed << std::setprecision(2) 
              << pair2.avg_time_us << " us";
    if (simple2.dof > 0 && pair2.dof > 0) {
        double ratio = pair2.avg_time_us / simple2.avg_time_us;
        std::cout << " (+" << std::fixed << std::setprecision(1) << 100.0*(ratio-1) << "%)\n";
    } else {
        std::cout << " (comparison unavailable)\n";
    }

    // 4-DOF / 4-link systems
    std::cout << "\nFour-DOF Systems (4 links):\n";
    std::cout << "  Simple Revolute:       " << std::fixed << std::setprecision(2) 
              << simple4.avg_time_us << " us\n";
    std::cout << "  RevolutePairWithRotor: " << std::fixed << std::setprecision(2) 
              << pair4.avg_time_us << " us";
    if (simple4.dof > 0 && pair4.dof > 0) {
        double ratio = pair4.avg_time_us / simple4.avg_time_us;
        std::cout << " (+" << std::fixed << std::setprecision(1) << 100.0*(ratio-1) << "%)\n";
    } else {
        std::cout << " (comparison unavailable)\n";
    }

    // 3-DOF / 6-link systems (3 links with triple mechanism = 3 DOF)
    std::cout << "\nTriple-Coupled Systems (6 links / 3 clusters):\n";
    std::cout << "  Simple Revolute (6):    " << std::fixed << std::setprecision(2) 
              << simple6.avg_time_us << " us\n";
    std::cout << "  RevoluteTripleWithRotor: " << std::fixed << std::setprecision(2) 
              << triple3.avg_time_us << " us";
    if (simple6.dof > 0 && triple3.dof > 0) {
        double ratio = triple3.avg_time_us / simple6.avg_time_us;
        std::cout << " (+" << std::fixed << std::setprecision(1) << 100.0*(ratio-1) << "%)\n";
    } else {
        std::cout << " (comparison unavailable)\n";
    }

    std::cout << "\n===========================================================================\n";
    std::cout << "Observations:\n";
    std::cout << "===========================================================================\n\n";
    std::cout << "1. Transmission Complexity: RevolutePairWithRotor and RevoluteTripleWithRotor\n";
    std::cout << "   include transmission modules (belt drives, gearboxes) that add computational\n";
    std::cout << "   overhead compared to simple revolute joints.\n\n";
    std::cout << "2. Constraint Complexity: The triple mechanism is more constrained than the pair,\n";
    std::cout << "   potentially leading to higher computational cost for derivative calculations.\n\n";
    std::cout << "3. Numerical Accuracy: Error magnitudes vary with joint complexity. More complex\n";
    std::cout << "   constraint systems may have different numerical conditioning.\n\n";
    std::cout << "4. DOF Scaling: Each cluster represents multiple physical joints but counts as\n";
    std::cout << "   fewer DOF due to internal constraints.\n\n";
    std::cout << "===========================================================================\n\n";

    return 0;
}
