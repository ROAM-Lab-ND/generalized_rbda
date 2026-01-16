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

// ============================================================================
// Inverse Dynamics Derivatives Scaling Benchmark
// ============================================================================
// This benchmark tests how computational cost scales with:
// 1. Number of links in a serial chain
// 2. Depth of a loop constraint in a two-link mechanism
// ============================================================================

struct ScalingResult {
    int num_links;
    int dof;
    double avg_time_us;
    double max_error_dq;
    double max_error_dqdot;
    double mean_error_dq;
    double mean_error_dqdot;
};

struct LoopResult {
    std::string mechanism_name;
    int num_loops;
    int dof;
    double avg_time_us;
    double max_error_dq;
    double max_error_dqdot;
};

// Test scaling for serial chains with increasing number of links
template<size_t N>
ScalingResult testSerialChainScaling() {
    RevoluteChainWithRotor<N, double> robot_real;
    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();

    const int nDOF = model_real.getNumDegreesOfFreedom();

    // Set random state
    ModelState<double> model_state;
    for (const auto& cluster : model_real.clusters()) {
        model_state.push_back(cluster->joint_->randomJointState());
    }
    model_real.setState(model_state);

    // Get state
    auto [q, qd] = model_real.getState();
    DVec<double> ydd = DVec<double>::Random(nDOF);

    // Compute analytical derivatives
    auto [dtau_dq_analytical, dtau_dqdot_analytical] = 
        model_real.firstOrderInverseDynamicsDerivatives(ydd);

    // Compute numerical derivatives using finite differences
    const double h = 1e-7;
    DMat<double> dtau_dq_numerical(nDOF, nDOF);
    DMat<double> dtau_dqdot_numerical(nDOF, nDOF);

    // Numerical dtau/dq
    for (int j = 0; j < nDOF; ++j) {
        DVec<double> q_plus = q;
        q_plus(j) += h;
        DVec<double> q_minus = q;
        q_minus(j) -= h;

        // Set perturbed state +h
        ModelState<double> state_plus;
        int idx = 0;
        for (const auto& cluster : model_real.clusters()) {
            JointState<double> js;
            js.position = q_plus.segment(idx, cluster->num_positions_);
            js.velocity = qd.segment(idx, cluster->num_velocities_);
            state_plus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model_real.setState(state_plus);
        DVec<double> tau_plus = model_real.inverseDynamics(ydd);

        // Set perturbed state -h
        ModelState<double> state_minus;
        idx = 0;
        for (const auto& cluster : model_real.clusters()) {
            JointState<double> js;
            js.position = q_minus.segment(idx, cluster->num_positions_);
            js.velocity = qd.segment(idx, cluster->num_velocities_);
            state_minus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model_real.setState(state_minus);
        DVec<double> tau_minus = model_real.inverseDynamics(ydd);

        dtau_dq_numerical.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    // Numerical dtau/dqdot
    for (int j = 0; j < nDOF; ++j) {
        DVec<double> qd_plus = qd;
        qd_plus(j) += h;
        DVec<double> qd_minus = qd;
        qd_minus(j) -= h;

        // Set perturbed state +h
        ModelState<double> state_plus;
        int idx = 0;
        for (const auto& cluster : model_real.clusters()) {
            JointState<double> js;
            js.position = q.segment(idx, cluster->num_positions_);
            js.velocity = qd_plus.segment(idx, cluster->num_velocities_);
            state_plus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model_real.setState(state_plus);
        DVec<double> tau_plus = model_real.inverseDynamics(ydd);

        // Set perturbed state -h
        ModelState<double> state_minus;
        idx = 0;
        for (const auto& cluster : model_real.clusters()) {
            JointState<double> js;
            js.position = q.segment(idx, cluster->num_positions_);
            js.velocity = qd_minus.segment(idx, cluster->num_velocities_);
            state_minus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model_real.setState(state_minus);
        DVec<double> tau_minus = model_real.inverseDynamics(ydd);

        dtau_dqdot_numerical.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    // Compute errors
    DMat<double> error_dq = dtau_dq_analytical - dtau_dq_numerical;
    DMat<double> error_dqdot = dtau_dqdot_analytical - dtau_dqdot_numerical;

    double max_error_dq = error_dq.cwiseAbs().maxCoeff();
    double max_error_dqdot = error_dqdot.cwiseAbs().maxCoeff();
    double mean_error_dq = error_dq.cwiseAbs().mean();
    double mean_error_dqdot = error_dqdot.cwiseAbs().mean();

    // Reset to original state for timing
    model_real.setState(model_state);

    // Benchmark timing
    const int WARMUP = 100;
    const int ITERATIONS = 1000;

    for (int i = 0; i < WARMUP; ++i) {
        auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ITERATIONS; ++i) {
        auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }
    auto end = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    double avg_time_us = total_us / ITERATIONS;

    return {static_cast<int>(N), nDOF, avg_time_us, max_error_dq, max_error_dqdot, 
            mean_error_dq, mean_error_dqdot};
}

// Test mechanism with loops
template<typename RobotType>
LoopResult testLoopMechanism(const std::string& name) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();

    const int nDOF = model.getNumDegreesOfFreedom();

    // Count loops (clusters with generic joints typically indicate loop constraints)
    int num_loops = 0;
    for (const auto& cluster : model.clusters()) {
        if (cluster->joint_->type() == ClusterJointTypes::Generic ||
            cluster->joint_->type() == ClusterJointTypes::FourBar) {
            num_loops++;
        }
    }

    // Set initial state to zero (safer for implicit constraints)
    ModelState<double> model_state;
    for (const auto& cluster : model.clusters()) {
        JointState<double> js;
        js.position = DVec<double>::Zero(cluster->num_positions_);
        js.velocity = DVec<double>::Zero(cluster->num_velocities_);
        model_state.push_back(js);
    }
    
    // Try to set a small random perturbation
    try {
        for (auto& js : model_state) {
            js.position += 0.01 * DVec<double>::Random(js.position.size());
            js.velocity += 0.01 * DVec<double>::Random(js.velocity.size());
        }
        model.setState(model_state);
    } catch (...) {
        // If random state fails, use zero state
        for (auto& js : model_state) {
            js.position.setZero();
            js.velocity.setZero();
        }
        model.setState(model_state);
    }

    auto [q, qd] = model.getState();
    DVec<double> ydd = DVec<double>::Random(nDOF);

    // Compute analytical derivatives
    auto [dtau_dq_analytical, dtau_dqdot_analytical] = 
        model.firstOrderInverseDynamicsDerivatives(ydd);

    // Compute numerical derivatives
    const double h = 1e-7;
    DMat<double> dtau_dq_numerical(nDOF, nDOF);
    DMat<double> dtau_dqdot_numerical(nDOF, nDOF);

    // Numerical dtau/dq
    for (int j = 0; j < nDOF; ++j) {
        DVec<double> q_plus = q;
        q_plus(j) += h;
        DVec<double> q_minus = q;
        q_minus(j) -= h;

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

        dtau_dq_numerical.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    // Numerical dtau/dqdot
    for (int j = 0; j < nDOF; ++j) {
        DVec<double> qd_plus = qd;
        qd_plus(j) += h;
        DVec<double> qd_minus = qd;
        qd_minus(j) -= h;

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
        int idx2 = 0;
        for (const auto& cluster : model.clusters()) {
            JointState<double> js;
            js.position = q.segment(idx2, cluster->num_positions_);
            js.velocity = qd_minus.segment(idx2, cluster->num_velocities_);
            state_minus.push_back(js);
            idx2 += cluster->num_velocities_;
        }
        model.setState(state_minus);
        DVec<double> tau_minus = model.inverseDynamics(ydd);

        dtau_dqdot_numerical.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    DMat<double> error_dq = dtau_dq_analytical - dtau_dq_numerical;
    DMat<double> error_dqdot = dtau_dqdot_analytical - dtau_dqdot_numerical;

    double max_error_dq = error_dq.cwiseAbs().maxCoeff();
    double max_error_dqdot = error_dqdot.cwiseAbs().maxCoeff();

    // Reset and benchmark
    model.setState(model_state);

    const int WARMUP = 100;
    const int ITERATIONS = 1000;

    for (int i = 0; i < WARMUP; ++i) {
        auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ITERATIONS; ++i) {
        auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }
    auto end = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    double avg_time_us = total_us / ITERATIONS;

    return {name, num_loops, nDOF, avg_time_us, max_error_dq, max_error_dqdot};
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Inverse Dynamics Derivatives Scaling Benchmark\n";
    std::cout << "===========================================================================\n\n";

    // Test 1: Serial chain scaling with simple revolute joints
    std::cout << "Test 1: Serial Chain Scaling - Simple Revolute Joints\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(8) << "Links"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<ScalingResult> chain_results;

    // Test chains of increasing length
    auto result_2 = testSerialChainScaling<2>();
    chain_results.push_back(result_2);
    std::cout << std::left << std::setw(8) << result_2.num_links
              << std::setw(6) << result_2.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_2.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_2.max_error_dq
              << std::setw(14) << result_2.max_error_dqdot
              << "\n";

    auto result_3 = testSerialChainScaling<3>();
    chain_results.push_back(result_3);
    std::cout << std::left << std::setw(8) << result_3.num_links
              << std::setw(6) << result_3.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_3.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_3.max_error_dq
              << std::setw(14) << result_3.max_error_dqdot
              << "\n";

    auto result_4 = testSerialChainScaling<4>();
    chain_results.push_back(result_4);
    std::cout << std::left << std::setw(8) << result_4.num_links
              << std::setw(6) << result_4.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_4.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_4.max_error_dq
              << std::setw(14) << result_4.max_error_dqdot
              << "\n";

    auto result_6 = testSerialChainScaling<6>();
    chain_results.push_back(result_6);
    std::cout << std::left << std::setw(8) << result_6.num_links
              << std::setw(6) << result_6.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_6.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_6.max_error_dq
              << std::setw(14) << result_6.max_error_dqdot
              << "\n";

    auto result_8 = testSerialChainScaling<8>();
    chain_results.push_back(result_8);
    std::cout << std::left << std::setw(8) << result_8.num_links
              << std::setw(6) << result_8.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_8.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_8.max_error_dq
              << std::setw(14) << result_8.max_error_dqdot
              << "\n";

    auto result_10 = testSerialChainScaling<10>();
    chain_results.push_back(result_10);
    std::cout << std::left << std::setw(8) << result_10.num_links
              << std::setw(6) << result_10.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_10.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_10.max_error_dq
              << std::setw(14) << result_10.max_error_dqdot
              << "\n";

    auto result_12 = testSerialChainScaling<12>();
    chain_results.push_back(result_12);
    std::cout << std::left << std::setw(8) << result_12.num_links
              << std::setw(6) << result_12.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_12.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_12.max_error_dq
              << std::setw(14) << result_12.max_error_dqdot
              << "\n";

    auto result_16 = testSerialChainScaling<16>();
    chain_results.push_back(result_16);
    std::cout << std::left << std::setw(8) << result_16.num_links
              << std::setw(6) << result_16.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_16.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_16.max_error_dq
              << std::setw(14) << result_16.max_error_dqdot
              << "\n";

    auto result_20 = testSerialChainScaling<20>();
    chain_results.push_back(result_20);
    std::cout << std::left << std::setw(8) << result_20.num_links
              << std::setw(6) << result_20.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << result_20.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << result_20.max_error_dq
              << std::setw(14) << result_20.max_error_dqdot
              << "\n";

    std::cout << "---------------------------------------------------------------------------\n\n";

    // Compute and display scaling characteristics
    std::cout << "Scaling Analysis:\n";
    std::cout << "---------------------------------------------------------------------------\n";
    
    // Time complexity: fit to O(n^k)
    if (chain_results.size() >= 3) {
        double log_ratio_time = std::log(chain_results.back().avg_time_us / chain_results[0].avg_time_us) /
                                std::log(static_cast<double>(chain_results.back().num_links) / chain_results[0].num_links);
        std::cout << "Time complexity exponent: O(n^" << std::fixed << std::setprecision(2) 
                  << log_ratio_time << ")\n";
        
        double log_ratio_error = std::log(chain_results.back().max_error_dq / chain_results[0].max_error_dq) /
                                 std::log(static_cast<double>(chain_results.back().num_links) / chain_results[0].num_links);
        std::cout << "Error growth exponent: O(n^" << std::fixed << std::setprecision(2) 
                  << log_ratio_error << ")\n";
        
        // Calculate per-link average time
        std::cout << "\nTime per link scaling:\n";
        for (size_t i = 0; i < std::min(size_t(5), chain_results.size()); ++i) {
            const auto& result = chain_results[i];
            double time_per_link = result.avg_time_us / result.num_links;
            std::cout << "  " << result.num_links << " links: "
                      << std::fixed << std::setprecision(2) << time_per_link 
                      << " us/link\n";
        }
        std::cout << "  ...\n";
        for (size_t i = std::max(size_t(5), chain_results.size() - 2); i < chain_results.size(); ++i) {
            const auto& result = chain_results[i];
            double time_per_link = result.avg_time_us / result.num_links;
            std::cout << "  " << result.num_links << " links: "
                      << std::fixed << std::setprecision(2) << time_per_link 
                      << " us/link\n";
        }
    }
    std::cout << "---------------------------------------------------------------------------\n\n";

    // Test 2: Mechanism with loops
    std::cout << "\nTest 2: Mechanism Loop Complexity\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(20) << "Mechanism"
              << std::setw(8) << "Loops"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<LoopResult> loop_results;

    // Test RevolutePair mechanisms (explicit loop constraints)
    // Each mechanism has N pairs, where each pair couples 2 revolute joints
    try {
        auto pair2_result = testLoopMechanism<RevolutePairChain<2, double>>("RevPairChain-2");
        loop_results.push_back(pair2_result);
        std::cout << std::left << std::setw(20) << pair2_result.mechanism_name
                  << std::setw(8) << pair2_result.num_loops
                  << std::setw(6) << pair2_result.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << pair2_result.avg_time_us
                  << std::setw(14) << std::scientific << std::setprecision(2) << pair2_result.max_error_dq
                  << std::setw(14) << pair2_result.max_error_dqdot
                  << "\n";
    } catch (const std::exception& e) {
        std::cout << std::left << std::setw(20) << "RevPairChain-2"
                  << std::setw(8) << "N/A"
                  << std::setw(6) << "N/A"
                  << std::setw(14) << "SKIPPED"
                  << std::setw(14) << "(error)"
                  << std::setw(14) << ""
                  << "\n";
    }

    try {
        auto pair4_result = testLoopMechanism<RevolutePairChain<4, double>>("RevPairChain-4");
        loop_results.push_back(pair4_result);
        std::cout << std::left << std::setw(20) << pair4_result.mechanism_name
                  << std::setw(8) << pair4_result.num_loops
                  << std::setw(6) << pair4_result.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << pair4_result.avg_time_us
                  << std::setw(14) << std::scientific << std::setprecision(2) << pair4_result.max_error_dq
                  << std::setw(14) << pair4_result.max_error_dqdot
                  << "\n";
    } catch (const std::exception& e) {
        std::cout << std::left << std::setw(20) << "RevPairChain-4"
                  << std::setw(8) << "N/A"
                  << std::setw(6) << "N/A"
                  << std::setw(14) << "SKIPPED"
                  << std::setw(14) << "(error)"
                  << std::setw(14) << ""
                  << "\n";
    }

    // Test serial chains for comparison (no loops)
    try {
        auto serial4_result = testLoopMechanism<RevoluteChainWithRotor<4, double>>("RevChain-4");
        loop_results.push_back(serial4_result);
        std::cout << std::left << std::setw(20) << serial4_result.mechanism_name
                  << std::setw(8) << serial4_result.num_loops
                  << std::setw(6) << serial4_result.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << serial4_result.avg_time_us
                  << std::setw(14) << std::scientific << std::setprecision(2) << serial4_result.max_error_dq
                  << std::setw(14) << serial4_result.max_error_dqdot
                  << "\n";
    } catch (const std::exception& e) {
        std::cout << std::left << std::setw(20) << "RevChain-4"
                  << std::setw(8) << "N/A"
                  << std::setw(6) << "N/A"
                  << std::setw(14) << "SKIPPED"
                  << std::setw(14) << "(error)"
                  << std::setw(14) << ""
                  << "\n";
    }

    try {
        auto serial8_result = testLoopMechanism<RevoluteChainWithRotor<8, double>>("RevChain-8");
        loop_results.push_back(serial8_result);
        std::cout << std::left << std::setw(20) << serial8_result.mechanism_name
                  << std::setw(8) << serial8_result.num_loops
                  << std::setw(6) << serial8_result.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << serial8_result.avg_time_us
                  << std::setw(14) << std::scientific << std::setprecision(2) << serial8_result.max_error_dq
                  << std::setw(14) << serial8_result.max_error_dqdot
                  << "\n";
    } catch (const std::exception& e) {
        std::cout << std::left << std::setw(20) << "RevChain-8"
                  << std::setw(8) << "N/A"
                  << std::setw(6) << "N/A"
                  << std::setw(14) << "SKIPPED"
                  << std::setw(14) << "(error)"
                  << std::setw(14) << ""
                  << "\n";
    }

    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << "\nLoop Mechanism Analysis:\n";
    std::cout << "RevolutePair mechanisms contain explicit loop constraints that couple\n";
    std::cout << "pairs of revolute joints. Each pair is a single 2-DOF cluster.\n";
    std::cout << "Compared to serial chains, loop mechanisms add overhead due to S_q\n";
    std::cout << "derivative computation. The S_q caching optimization reduces this by\n";
    std::cout << "avoiding repeated CasADi evaluations.\n";
    if (loop_results.size() >= 2) {
        std::cout << "\nComparison (4 DOF systems):\n";
        for (const auto& result : loop_results) {
            if (result.dof == 4) {
                std::cout << "  " << result.mechanism_name << " (" << result.num_loops << " loops): "
                          << std::fixed << std::setprecision(2) << result.avg_time_us << " us\n";
            }
        }
    }
    std::cout << "---------------------------------------------------------------------------\n\n";

    std::cout << "===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n";

    return 0;
}
