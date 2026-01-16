#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>
#include <filesystem>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;
namespace fs = std::filesystem;

struct ScalingResult {
    std::string model_name;
    int n_links;
    int loop_depth;
    int dof;
    double avg_time_us;
    double max_error_dq;
    double max_error_dqdot;
};

// Test a loaded URDF model
ScalingResult testParallelChainModel(const std::string& urdf_path, const std::string& model_name, 
                                     int& n_links_out, int& loop_depth_out) {
    try {
        // Parse model name to extract n_links and loop_depth
        // Format: ParallelChain_N_K
        size_t pos1 = model_name.find_last_of('_');
        size_t pos2 = model_name.find_last_of('_', pos1 - 1);
        loop_depth_out = std::stoi(model_name.substr(pos1 + 1));
        n_links_out = std::stoi(model_name.substr(pos2 + 1, pos1 - pos2 - 1));
        
        // Load model from URDF
        ClusterTreeModel<double> model(urdf_path);
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
            
            // Set perturbed state +h
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
            
            // Set perturbed state -h
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
            
            // Set perturbed state +h
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
            
            // Set perturbed state -h
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

        ScalingResult result;
        result.model_name = model_name;
        result.n_links = n_links_out;
        result.loop_depth = loop_depth_out;
        result.dof = nDOF;
        result.avg_time_us = avg_time_us;
        result.max_error_dq = error_dq.maxCoeff();
        result.max_error_dqdot = error_dqdot.maxCoeff();

        return result;
    } catch (const std::exception& e) {
        std::cerr << "Error testing model " << model_name << ": " << e.what() << "\n";
        ScalingResult result;
        result.model_name = model_name + " (ERROR)";
        result.dof = -1;
        result.avg_time_us = 0;
        result.max_error_dq = 0;
        result.max_error_dqdot = 0;
        return result;
    }
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Parallel Chain Scaling Benchmark\n";
    std::cout << "Two independent chains with loop constraints at varying depths\n";
    std::cout << "===========================================================================\n\n";

    // Test parallel chains with 4 links
    std::cout << "Test 1: Four-Link Parallel Chains - Loop at Varying Depths\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(20) << "Model"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<ScalingResult> results_4;
    for (int depth = 1; depth <= 4; ++depth) {
        std::string urdf_path = "/source/generalized_rbda/robot-models/parallel_chains/parallel_chain_4_" + 
                                std::to_string(depth) + ".urdf";
        std::string model_name = "ParallelChain_4_" + std::to_string(depth);
        
        int n_links = 0, loop_depth = 0;
        auto result = testParallelChainModel(urdf_path, model_name, n_links, loop_depth);
        results_4.push_back(result);
        
        if (result.dof > 0) {
            std::cout << std::left << std::setw(20) << result.model_name
                      << std::setw(6) << result.dof
                      << std::setw(14) << std::fixed << std::setprecision(2) << result.avg_time_us
                      << std::setw(14) << std::scientific << std::setprecision(2) << result.max_error_dq
                      << std::setw(14) << result.max_error_dqdot
                      << "\n";
        } else {
            std::cout << std::left << std::setw(20) << result.model_name
                      << std::setw(6) << "N/A"
                      << std::setw(14) << "FAILED"
                      << std::setw(14) << ""
                      << std::setw(14) << ""
                      << "\n";
        }
    }

    std::cout << "---------------------------------------------------------------------------\n\n";

    // Analyze depth effect (for models that succeed)
    auto valid_results = results_4;
    valid_results.erase(
        std::remove_if(valid_results.begin(), valid_results.end(),
                      [](const ScalingResult& r) { return r.dof < 0; }),
        valid_results.end()
    );
    
    if (valid_results.size() >= 2) {
        std::cout << "Analysis of Loop Depth Effect (4-link chains):\n";
        std::cout << "---------------------------------------------------------------------------\n";
        std::cout << "Depth 1 (early):   " << std::fixed << std::setprecision(2) 
                  << results_4[0].avg_time_us << " us\n";
        std::cout << "Depth 4 (late):    " << std::fixed << std::setprecision(2) 
                  << results_4[3].avg_time_us << " us\n";
        double ratio = results_4[3].avg_time_us / results_4[0].avg_time_us;
        std::cout << "Ratio (late/early): " << std::fixed << std::setprecision(2) << ratio << "x\n";
        std::cout << "\nObservation: Loop constraint position affects computational cost due to\n";
        std::cout << "the order of operations in forward dynamics and constraint handling.\n\n";
    }

    // Test parallel chains with 6 links
    std::cout << "\n===========================================================================\n";
    std::cout << "Test 2: Six-Link Parallel Chains - Loop at Varying Depths\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(20) << "Model"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << "---------------------------------------------------------------------------\n";

    std::vector<ScalingResult> results_6;
    for (int depth = 1; depth <= 6; ++depth) {
        std::string urdf_path = "/source/generalized_rbda/robot-models/parallel_chains/parallel_chain_6_" + 
                                std::to_string(depth) + ".urdf";
        std::string model_name = "ParallelChain_6_" + std::to_string(depth);
        
        int n_links = 0, loop_depth = 0;
        auto result = testParallelChainModel(urdf_path, model_name, n_links, loop_depth);
        results_6.push_back(result);
        
        if (result.dof > 0) {
            std::cout << std::left << std::setw(20) << result.model_name
                      << std::setw(6) << result.dof
                      << std::setw(14) << std::fixed << std::setprecision(2) << result.avg_time_us
                      << std::setw(14) << std::scientific << std::setprecision(2) << result.max_error_dq
                      << std::setw(14) << result.max_error_dqdot
                      << "\n";
        } else {
            std::cout << std::left << std::setw(20) << result.model_name
                      << std::setw(6) << "N/A"
                      << std::setw(14) << "FAILED"
                      << std::setw(14) << ""
                      << std::setw(14) << ""
                      << "\n";
        }
    }

    std::cout << "---------------------------------------------------------------------------\n\n";

    // Summary
    std::cout << "\n===========================================================================\n";
    std::cout << "Summary\n";
    std::cout << "===========================================================================\n\n";
    std::cout << "This benchmark tests how the computational cost of inverse dynamics derivatives\n";
    std::cout << "changes when the loop constraint connecting two parallel chains is moved further\n";
    std::cout << "down the kinematic chain.\n\n";
    std::cout << "Model Structure:\n";
    std::cout << "  - Two parallel kinematic chains\n";
    std::cout << "  - Each with N revolute joints (1 DOF per joint)\n";
    std::cout << "  - Connected by a single loop constraint at depth K\n";
    std::cout << "  - Total DOF = 2*N - 1 (one constraint reduces 2*N by 1)\n\n";
    std::cout << "Expected Behavior:\n";
    std::cout << "  - Early loops (K small): Fewer DOF processed before constraint\n";
    std::cout << "  - Late loops (K large): More DOF processed before constraint\n";
    std::cout << "  - Timing may vary based on caching and memory access patterns\n";
    std::cout << "===========================================================================\n\n";

    return 0;
}
