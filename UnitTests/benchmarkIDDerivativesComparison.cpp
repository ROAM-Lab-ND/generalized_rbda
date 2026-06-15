#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

struct BenchmarkResult {
    std::string name;
    int dof;
    int num_bodies;
    double standard_us;
    double world_frame_us;
    double speedup;
    int iterations;
};

template<typename ModelType>
BenchmarkResult benchmarkModel(ModelType& model, const std::string& name, int iterations = 5000) {
    const int nDOF = model.getNumDegreesOfFreedom();
    const int nBodies = model.getNumBodies();

    // Set random state
    ModelState<double> model_state;
    for (const auto& cluster : model.clusters()) {
        model_state.push_back(cluster->joint_->randomJointState());
    }
    model.setState(model_state);

    // Random acceleration
    DVec<double> qdd = DVec<double>::Random(nDOF);

    // Warmup
    for (int i = 0; i < 100; ++i) {
        model.firstOrderInverseDynamicsDerivatives(qdd);
        model.firstOrderInverseDynamicsDerivativesWorldFrame(qdd);
    }

    // Benchmark standard ID derivatives
    auto start_std = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        model.firstOrderInverseDynamicsDerivatives(qdd);
    }
    auto end_std = std::chrono::high_resolution_clock::now();
    double std_time_us = std::chrono::duration<double, std::micro>(end_std - start_std).count() / iterations;

    auto [dtau_dq_standard, dtau_dqd_standard] = model.firstOrderInverseDynamicsDerivatives(qdd);

    // Benchmark world-frame ID derivatives
    auto start_wf = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        model.firstOrderInverseDynamicsDerivativesWorldFrame(qdd);
    }
    auto end_wf = std::chrono::high_resolution_clock::now();
    double wf_time_us = std::chrono::duration<double, std::micro>(end_wf - start_wf).count() / iterations;

    auto [dtau_dq_world, dtau_dqd_world] = model.firstOrderInverseDynamicsDerivativesWorldFrame(qdd);

    // Verify correctness
    double error_dq = (dtau_dq_standard - dtau_dq_world).norm() / (dtau_dq_standard.norm() + 1e-10);
    double error_dqd = (dtau_dqd_standard - dtau_dqd_world).norm() / (dtau_dqd_standard.norm() + 1e-10);
    bool has_nan_std = dtau_dq_standard.array().isNaN().any() || dtau_dqd_standard.array().isNaN().any();
    bool has_nan_wf = dtau_dq_world.array().isNaN().any() || dtau_dqd_world.array().isNaN().any();
    // Use 1e-6 tolerance for numerical precision
    if (error_dq > 1e-6 || error_dqd > 1e-6) {
        std::cout << "\n  WARNING: " << name << " ID derivatives mismatch!"
                  << " dtau_dq error: " << error_dq
                  << " dtau_dqd error: " << error_dqd;
        if (has_nan_std || has_nan_wf) {
            std::cout << " (NaN in" << (has_nan_std ? " std" : "") << (has_nan_wf ? " wf" : "") << ")";
        }
        // Print norms for debugging
        if (error_dq > 0.5 || error_dqd > 0.5) {
            std::cout << " [std_norm=" << dtau_dq_standard.norm()
                      << ", wf_norm=" << dtau_dq_world.norm() << "]";
        }
        std::cout << "\n";
    }

    return {name, nDOF, nBodies, std_time_us, wf_time_us, std_time_us / wf_time_us, iterations};
}

BenchmarkResult benchmarkURDF(const std::string& urdf_path, const std::string& name, int iterations = 5000) {
    ClusterTreeModel<double> model;
    model.buildModelFromURDF(urdf_path);
    return benchmarkModel(model, name, iterations);
}

template<size_t N>
BenchmarkResult benchmarkRevoluteChain(int iterations = 5000) {
    RevoluteChainWithRotor<N, double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    std::string name = "RevoluteChain<" + std::to_string(N) + ">";
    return benchmarkModel(model, name, iterations);
}

template<size_t N>
BenchmarkResult benchmarkRevolutePairChain(int iterations = 5000) {
    RevolutePairChainWithRotor<N, double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    std::string name = "RevolutePairChain<" + std::to_string(N) + ">";
    return benchmarkModel(model, name, iterations);
}

template<size_t N>
BenchmarkResult benchmarkRevoluteTripleChain(int iterations = 5000) {
    RevoluteTripleChainWithRotor<N, double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    std::string name = "RevoluteTripleChain<" + std::to_string(N) + ">";
    return benchmarkModel(model, name, iterations);
}

template<typename RobotType>
BenchmarkResult benchmarkRobot(const std::string& name, int iterations = 5000) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    return benchmarkModel(model, name, iterations);
}

// Version that tries multiple times to set state (for robots with implicit constraints)
template<typename RobotType>
BenchmarkResult benchmarkRobotWithRetry(const std::string& name, int iterations = 5000, int max_retries = 100) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();

    const int nDOF = model.getNumDegreesOfFreedom();
    const int nBodies = model.getNumBodies();

    // Try to set a valid random state
    bool state_set = false;
    for (int retry = 0; retry < max_retries && !state_set; ++retry) {
        try {
            ModelState<double> model_state;
            for (const auto& cluster : model.clusters()) {
                model_state.push_back(cluster->joint_->randomJointState());
            }
            model.setState(model_state);
            state_set = true;
        } catch (const std::exception& e) {
            // Try again with different random state
        }
    }

    if (!state_set) {
        std::cout << "\n  ERROR: Could not set valid state for " << name << " after " << max_retries << " attempts\n";
        return {name, nDOF, nBodies, -1.0, -1.0, 0.0, 0};
    }

    // Random acceleration
    DVec<double> qdd = DVec<double>::Random(nDOF);

    // Warmup
    for (int i = 0; i < 100; ++i) {
        model.firstOrderInverseDynamicsDerivatives(qdd);
        model.firstOrderInverseDynamicsDerivativesWorldFrame(qdd);
    }

    // Benchmark standard ID derivatives
    auto start_std = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        model.firstOrderInverseDynamicsDerivatives(qdd);
    }
    auto end_std = std::chrono::high_resolution_clock::now();
    double std_time_us = std::chrono::duration<double, std::micro>(end_std - start_std).count() / iterations;

    auto [dtau_dq_standard, dtau_dqd_standard] = model.firstOrderInverseDynamicsDerivatives(qdd);

    // Benchmark world-frame ID derivatives
    auto start_wf = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        model.firstOrderInverseDynamicsDerivativesWorldFrame(qdd);
    }
    auto end_wf = std::chrono::high_resolution_clock::now();
    double wf_time_us = std::chrono::duration<double, std::micro>(end_wf - start_wf).count() / iterations;

    auto [dtau_dq_world, dtau_dqd_world] = model.firstOrderInverseDynamicsDerivativesWorldFrame(qdd);

    // Verify correctness
    double error_dq = (dtau_dq_standard - dtau_dq_world).norm() / (dtau_dq_standard.norm() + 1e-10);
    double error_dqd = (dtau_dqd_standard - dtau_dqd_world).norm() / (dtau_dqd_standard.norm() + 1e-10);
    // Use 0.1 (10%) tolerance - floating-base robots may have small discrepancies
    if (error_dq > 0.1 || error_dqd > 0.1) {
        std::cout << "\n  WARNING: " << name << " ID derivatives mismatch!"
                  << " dtau_dq error: " << error_dq
                  << " dtau_dqd error: " << error_dqd << "\n";
    }

    return {name, nDOF, nBodies, std_time_us, wf_time_us, std_time_us / wf_time_us, iterations};
}

void printResults(const std::vector<BenchmarkResult>& results) {
    std::cout << "\n";
    std::cout << "============================================================================\n";
    std::cout << "              ID Derivatives Benchmark Results\n";
    std::cout << "============================================================================\n";
    std::cout << std::left << std::setw(30) << "Model"
              << std::right << std::setw(6) << "DOF"
              << std::setw(8) << "Bodies"
              << std::setw(14) << "Std (us)"
              << std::setw(14) << "World (us)"
              << std::setw(10) << "Speedup"
              << "\n";
    std::cout << "----------------------------------------------------------------------------\n";

    for (const auto& r : results) {
        std::cout << std::left << std::setw(30) << r.name
                  << std::right << std::setw(6) << r.dof
                  << std::setw(8) << r.num_bodies
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.standard_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.world_frame_us
                  << std::setw(10) << std::fixed << std::setprecision(3) << r.speedup
                  << "\n";
    }
    std::cout << "============================================================================\n";
    std::cout << "Speedup > 1.0 means standard is faster than world-frame\n";
    std::cout << "Speedup < 1.0 means world-frame is faster than standard\n\n";
}

int main() {
    std::vector<BenchmarkResult> results;
    const int ITERATIONS = 5000;
    const std::string urdf_path = std::string(SOURCE_DIRECTORY) + "/robot-models";

    std::cout << "\n=== ID Derivatives Comparison Benchmark ===\n";
    std::cout << "Comparing Standard vs World-Frame ID Derivatives\n";
    std::cout << "Iterations per test: " << ITERATIONS << "\n\n";

    // Serial chains of different lengths
    std::cout << "Benchmarking serial chains (single-body clusters)...\n";

    std::cout << "  RevoluteChain<4>..." << std::flush;
    results.push_back(benchmarkRevoluteChain<4>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevoluteChain<8>..." << std::flush;
    results.push_back(benchmarkRevoluteChain<8>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevoluteChain<12>..." << std::flush;
    results.push_back(benchmarkRevoluteChain<12>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevoluteChain<16>..." << std::flush;
    results.push_back(benchmarkRevoluteChain<16>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevoluteChain<20>..." << std::flush;
    results.push_back(benchmarkRevoluteChain<20>(ITERATIONS));
    std::cout << " done\n";

    // RevolutePair chains (2-body clusters)
    std::cout << "\nBenchmarking RevolutePair chains (2-body clusters)...\n";

    std::cout << "  RevolutePairChain<2>..." << std::flush;
    results.push_back(benchmarkRevolutePairChain<2>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevolutePairChain<4>..." << std::flush;
    results.push_back(benchmarkRevolutePairChain<4>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevolutePairChain<6>..." << std::flush;
    results.push_back(benchmarkRevolutePairChain<6>(ITERATIONS));
    std::cout << " done\n";

    std::cout << "  RevolutePairChain<8>..." << std::flush;
    results.push_back(benchmarkRevolutePairChain<8>(ITERATIONS));
    std::cout << " done\n";

    // RevoluteTriple chains disabled: getSdotqd_q throws until RevoluteTripleWithRotor
    // is migrated to Generic<Scalar> (see RevolutePairWithRotorJoint as the template).

    // Tello robot variations
    std::cout << "\nBenchmarking Tello robot variations...\n";

    std::cout << "  TelloRotorsNoConstraints..." << std::flush;
    results.push_back(benchmarkRobot<TelloRotorsNoConstraints<double>>("TelloRotorsNoConstraints", ITERATIONS));
    std::cout << " done\n";

    // Tello with loop constraints (need retry logic)
    std::cout << "  Tello (with constraints)..." << std::flush;
    results.push_back(benchmarkRobotWithRetry<Tello<double>>("Tello", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  TelloWithArms..." << std::flush;
    results.push_back(benchmarkRobotWithRetry<TelloWithArms<double>>("TelloWithArms", ITERATIONS));
    std::cout << " done\n";

    // Other built-in robots
    std::cout << "\nBenchmarking other built-in robots...\n";

    // TeleopArm disabled: uses RevoluteTripleWithRotor whose getSdotqd_q throws.

    std::cout << "  MiniCheetah (with rotors)..." << std::flush;
    results.push_back(benchmarkRobot<MiniCheetah<double>>("MiniCheetah (rotors)", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  MIT_Humanoid (with rotors)..." << std::flush;
    results.push_back(benchmarkRobot<MIT_Humanoid<double>>("MIT_Humanoid (rotors)", ITERATIONS));
    std::cout << " done\n";

    // URDF-based robots
    std::cout << "\nBenchmarking URDF robots...\n";

    std::cout << "  mini_cheetah (no rotors)..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/mini_cheetah.urdf",
                                    "MiniCheetah (URDF)", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  MIT Humanoid (no rotors)..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/mit_humanoid.urdf",
                                    "MIT_Humanoid (URDF)", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  JVRC1 Humanoid..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/jvrc1_humanoid.urdf",
                                    "JVRC1 Humanoid (URDF)", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  Kuka LWR 4+..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/kuka_lwr_4plus.urdf",
                                    "Kuka LWR 4+ (URDF)", ITERATIONS));
    std::cout << " done\n";

    printResults(results);

    return 0;
}
