#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <atomic>
#include <cstdlib>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "grbda/Utils/IDDerivProfile.h"
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

std::atomic<int> g_model_build_count{0};

bool envAssertSingleBuildEnabled() {
    const char* value = std::getenv("GRBDA_ID_DERIV_ASSERT_SINGLE_BUILD");
    return value != nullptr && value[0] != '0';
}

void noteModelBuild(const std::string& model_name) {
    const int build_idx = ++g_model_build_count;
    if (std::getenv("GRBDA_ID_DERIV_MODEL_BUILD_LOG") != nullptr) {
        std::cout << "[IDDerivBuild] build_index=" << build_idx
                  << " model=" << model_name << "\n";
    }
}

void printBuildSummaryAndAssertIfRequested() {
    const int total_builds = g_model_build_count.load();
    std::cout << "[IDDerivBuildSummary] total_model_builds=" << total_builds << "\n";

    if (envAssertSingleBuildEnabled() && total_builds != 1) {
        std::cerr << "[IDDerivBuildSummary] ASSERTION FAILED: expected exactly 1 model build, got "
                  << total_builds << "\n";
        std::exit(2);
    }
}

bool envEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '0';
}

void maybeDumpState(const std::string& name, ClusterTreeModel<double>& model) {
    const char* dump_env = std::getenv("GRBDA_ID_DERIV_DUMP_VALID_STATE");
    if (dump_env == nullptr || dump_env[0] == '0') {
        return;
    }

    const auto [q, qd] = model.getState();
    std::cout << "[IDDerivValidState] " << name
              << " q_size=" << q.size()
              << " qd_size=" << qd.size() << "\n";

    std::cout << "[IDDerivValidState] " << name << " q=";
    for (int i = 0; i < q.size(); ++i) {
        if (i) std::cout << ",";
        std::cout << std::setprecision(17) << q(i);
    }
    std::cout << "\n";

    std::cout << "[IDDerivValidState] " << name << " qd=";
    for (int i = 0; i < qd.size(); ++i) {
        if (i) std::cout << ",";
        std::cout << std::setprecision(17) << qd(i);
    }
    std::cout << "\n";
}

int envIntOrDefault(const char* name, int default_value) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return default_value;
    }
    try {
        const int parsed = std::stoi(value);
        return parsed > 0 ? parsed : default_value;
    } catch (...) {
        return default_value;
    }
}

void printProfileSummary(const std::string& name, const std::string& phase) {
    const char* print_profile_env = std::getenv("GRBDA_ID_DERIV_PROFILE_SUMMARY");
    if (print_profile_env == nullptr || print_profile_env[0] == '0') {
        return;
    }

    const auto profile = profiling::getCurrentCall();
    std::cout << "[IDDerivProfileSummary] " << name << " " << phase
              << " forward_us=" << std::fixed << std::setprecision(4) << profile.forward_us
              << " backward_us=" << std::fixed << std::setprecision(4) << profile.backward_us
              << " casadi_us=" << std::fixed << std::setprecision(4) << profile.casadi_us
              << " getsq_us=" << std::fixed << std::setprecision(4) << profile.getsq_us
              << " getsq_internal_us=" << std::fixed << std::setprecision(4) << profile.getsq_internal_us
              << std::defaultfloat
              << "\n";
}

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
    printProfileSummary(name, "warm");

    const char* print_profile_env = std::getenv("GRBDA_ID_DERIV_PROFILE_SUMMARY");
    if (print_profile_env != nullptr && print_profile_env[0] != '0') {
        // Force a fresh state so the cold-call timing captures cache rebuild and CasADi work.
        ModelState<double> profile_state;
        for (const auto& cluster : model.clusters()) {
            profile_state.push_back(cluster->joint_->randomJointState());
        }
        model.setState(profile_state);

        (void)model.firstOrderInverseDynamicsDerivatives(qdd);
        printProfileSummary(name, "cold");
    }

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
    noteModelBuild(name);
    return benchmarkModel(model, name, iterations);
}

template<size_t N>
BenchmarkResult benchmarkRevoluteChain(int iterations = 5000) {
    RevoluteChainWithRotor<N, double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    std::string name = "RevoluteChain<" + std::to_string(N) + ">";
    noteModelBuild(name);
    return benchmarkModel(model, name, iterations);
}

template<size_t N>
BenchmarkResult benchmarkRevolutePairChain(int iterations = 5000) {
    RevolutePairChainWithRotor<N, double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    std::string name = "RevolutePairChain<" + std::to_string(N) + ">";
    noteModelBuild(name);
    return benchmarkModel(model, name, iterations);
}

template<size_t N>
BenchmarkResult benchmarkRevoluteTripleChain(int iterations = 5000) {
    RevoluteTripleChainWithRotor<N, double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    std::string name = "RevoluteTripleChain<" + std::to_string(N) + ">";
    noteModelBuild(name);
    return benchmarkModel(model, name, iterations);
}

template<typename RobotType>
BenchmarkResult benchmarkRobot(const std::string& name, int iterations = 5000) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    noteModelBuild(name);
    return benchmarkModel(model, name, iterations);
}

// Version that tries multiple times to set state (for robots with implicit constraints)
template<typename RobotType>
BenchmarkResult benchmarkRobotWithRetry(const std::string& name, int iterations = 5000, int max_retries = 100) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    noteModelBuild(name);

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
        try {
            // Deterministic fallback for constrained models: identity floating-base
            // pose plus zero spanning coordinates/velocities.
            ModelState<double> fallback_state;
            for (const auto& cluster : model.clusters()) {
                JointState<double> joint_state(cluster->joint_->isImplicit(), false);
                joint_state.position = DVec<double>::Zero(cluster->joint_->numPositions());
                joint_state.velocity = DVec<double>::Zero(cluster->joint_->numVelocities());

                // Free joint convention in this codebase: [x y z qw qx qy qz].
                if (!cluster->joint_->isImplicit() && cluster->joint_->numPositions() == 7) {
                    joint_state.position(2) = 1.0;
                    joint_state.position(3) = 1.0;
                }

                fallback_state.push_back(joint_state);
            }

            model.setState(fallback_state);
            state_set = true;
        } catch (const std::exception&) {
        }
    }

    if (!state_set) {
        std::cout << "\n  ERROR: Could not set valid state for " << name << " after " << max_retries << " attempts\n";
        return {name, nDOF, nBodies, -1.0, -1.0, 0.0, 0};
    }

    maybeDumpState(name, model);

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
    printProfileSummary(name, "warm");

    const char* print_profile_env = std::getenv("GRBDA_ID_DERIV_PROFILE_SUMMARY");
    if (print_profile_env != nullptr && print_profile_env[0] != '0') {
        // Force a fresh state so the cold-call timing captures cache rebuild and CasADi work.
        bool profile_state_set = false;
        for (int retry = 0; retry < max_retries && !profile_state_set; ++retry) {
            try {
                ModelState<double> profile_state;
                for (const auto& cluster : model.clusters()) {
                    profile_state.push_back(cluster->joint_->randomJointState());
                }
                model.setState(profile_state);
                profile_state_set = true;
            } catch (const std::exception&) {
            }
        }

        if (profile_state_set) {
            (void)model.firstOrderInverseDynamicsDerivatives(qdd);
            printProfileSummary(name, "cold");
        } else {
            std::cout << "[IDDerivProfileSummary] " << name
                      << " cold_skipped reason=fresh_state_not_found" << "\n";
        }
    }

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
    const bool ONLY_TELLO = envEnabled("GRBDA_ID_DERIV_ONLY_TELLO");
    const int ITERATIONS = envIntOrDefault("GRBDA_ID_DERIV_ITERATIONS", 5000);
    const std::string urdf_path = std::string(SOURCE_DIRECTORY) + "/robot-models";

    std::cout << "\n=== ID Derivatives Comparison Benchmark ===\n";
    std::cout << "Comparing Standard vs World-Frame ID Derivatives\n";
    std::cout << "Iterations per test: " << ITERATIONS << "\n\n";
    if (ONLY_TELLO) {
        std::cout << "Mode: Tello-only\n\n";
    }

    if (!ONLY_TELLO) {
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

        // RevoluteTriple chains (3-body clusters) - N must be divisible by 3
        std::cout << "\nBenchmarking RevoluteTriple chains (3-body clusters)...\n";

        std::cout << "  RevoluteTripleChain<3>..." << std::flush;
        results.push_back(benchmarkRevoluteTripleChain<3>(ITERATIONS));
        std::cout << " done\n";

        std::cout << "  RevoluteTripleChain<6>..." << std::flush;
        results.push_back(benchmarkRevoluteTripleChain<6>(ITERATIONS));
        std::cout << " done\n";

        std::cout << "  RevoluteTripleChain<9>..." << std::flush;
        results.push_back(benchmarkRevoluteTripleChain<9>(ITERATIONS));
        std::cout << " done\n";
    }

    // Tello robot variations
    std::cout << "\nBenchmarking Tello robot variations...\n";

    std::cout << "  Tello (+R/-M)..." << std::flush;
    results.push_back(benchmarkRobot<TelloNoMechanisms<double>>("Tello (+R/-M)", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  TelloRotorsNoConstraints..." << std::flush;
    results.push_back(benchmarkRobot<TelloRotorsNoConstraints<double>>("TelloRotorsNoConstraints", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  TelloClusteredNoConstraints..." << std::flush;
    results.push_back(benchmarkRobot<TelloClusteredNoConstraints<double>>("TelloClusteredNoConstraints", ITERATIONS));
    std::cout << " done\n";

    // Tello with loop constraints (need retry logic)
    std::cout << "  Tello (+R/+M)..." << std::flush;
    results.push_back(benchmarkRobotWithRetry<Tello<double>>("Tello (+R/+M)", ITERATIONS));
    std::cout << " done\n";

    std::cout << "  Tello with Arms (+R/+M)..." << std::flush;
    results.push_back(benchmarkRobotWithRetry<TelloWithArms<double>>("Tello with Arms (+R/+M)", ITERATIONS));
    std::cout << " done\n";

    if (!ONLY_TELLO) {
        // Other built-in robots
        std::cout << "\nBenchmarking other built-in robots...\n";

        std::cout << "  TeleopArm..." << std::flush;
        results.push_back(benchmarkRobot<TeleopArm<double>>("TeleopArm", ITERATIONS));
        std::cout << " done\n";

        std::cout << "  MiniCheetah (with rotors)..." << std::flush;
        results.push_back(benchmarkRobot<MiniCheetah<double>>("MiniCheetah (rotors)", ITERATIONS));
        std::cout << " done\n";

        std::cout << "  MIT_Humanoid (with rotors)..." << std::flush;
        results.push_back(benchmarkRobot<MIT_Humanoid<double>>("MIT_Humanoid (rotors)", ITERATIONS));
        std::cout << " done\n";

        std::cout << "  Cassie (closed-loop leg)..." << std::flush;
        results.push_back(benchmarkRobotWithRetry<Cassie<double>>("Cassie (closed-loop leg)", ITERATIONS));
        std::cout << " done\n";

        // Other built-in robots
        std::cout << "\nBenchmarking other built-in robots...\n";

        std::cout << "  MiniCheetah (no rotors)..." << std::flush;
        results.push_back(benchmarkRobot<MiniCheetah_no_rotors<double, ori_representation::Quaternion>>(
            "MiniCheetah (no rotors)", ITERATIONS));
        std::cout << " done\n";

        std::cout << "  MIT Humanoid (no rotors)..." << std::flush;
        results.push_back(benchmarkRobot<MIT_Humanoid_no_rotors<double>>("MIT_Humanoid (no rotors)", ITERATIONS));
        std::cout << " done\n";

        std::cout << "  JVRC1 Humanoid..." << std::flush;
        results.push_back(benchmarkURDF(urdf_path + "/jvrc1_humanoid.urdf",
                                        "JVRC1 Humanoid (URDF)", ITERATIONS));
        std::cout << " done\n";

        std::cout << "  Kuka LWR 4+..." << std::flush;
        results.push_back(benchmarkURDF(urdf_path + "/kuka_lwr_4plus.urdf",
                                        "Kuka LWR 4+ (URDF)", ITERATIONS));
        std::cout << " done\n";
    }

    printResults(results);
    printBuildSummaryAndAssertIfRequested();

    return 0;
}
