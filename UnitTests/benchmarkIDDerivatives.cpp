#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdlib>
#include <stdexcept>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

bool envFlagEnabled(const char* name, bool fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    return raw[0] != '0';
}

struct BenchmarkResult {
    std::string name;
    int dof;
    double avg_time_us;
    int iterations;
};

BenchmarkResult benchmarkModel(ClusterTreeModel<double>& model, const std::string& name, int iterations) {
    const int nDOF = model.getNumDegreesOfFreedom();

    // Find a valid model state. Some robots can throw for unlucky random samples,
    // so retry full-state sampling until setState and a probe derivative call succeed.
    DVec<double> ydd = DVec<double>::Random(nDOF);
    bool valid_state_found = false;
    for (int attempt = 0; attempt < 512; ++attempt) {
        ModelState<double> model_state;
        model_state.reserve(model.clusters().size());

        bool joint_sampling_ok = true;
        for (const auto& cluster : model.clusters()) {
            try {
                model_state.push_back(cluster->joint_->randomJointState());
            } catch (const std::exception&) {
                joint_sampling_ok = false;
                break;
            }
        }

        if (!joint_sampling_ok) {
            continue;
        }

        try {
            model.setState(model_state);
            auto [dtau_dq_check, dtau_dqdot_check] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq_check;
            (void)dtau_dqdot_check;
            valid_state_found = true;
            break;
        } catch (const std::exception&) {
            // Retry with a new sampled state.
        }
    }
    if (!valid_state_found) {
        throw std::runtime_error("Failed to find valid benchmark state for " + name);
    }

    // Warmup
    for (int i = 0; i < 100; ++i) {
        auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }

    // Timed iterations
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }
    auto end = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(end - start).count();

    return {name, nDOF, total_us / iterations, iterations};
}

template<typename RobotType>
BenchmarkResult benchmarkRobot(const std::string& name, int iterations = 1000) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    return benchmarkModel(model, name, iterations);
}

BenchmarkResult benchmarkURDF(const std::string& urdf_path, const std::string& name, int iterations = 1000) {
    ClusterTreeModel<double> model;
    model.buildModelFromURDF(urdf_path);
    return benchmarkModel(model, name, iterations);
}

int main() {
    std::vector<BenchmarkResult> results;
    const int ITERATIONS = 1000;
    const std::string urdf_path = std::string(SOURCE_DIRECTORY) + "/robot-models";

    std::cout << "\nLoading and benchmarking robots...\n";
    std::cout << "URDF path: " << urdf_path << "\n\n";

    // Mini Cheetah with rotors
    std::cout << "  Benchmarking MiniCheetah (with rotors)..." << std::flush;
    results.push_back(benchmarkRobot<MiniCheetah<double, ori_representation::Quaternion>>(
        "MiniCheetah (with rotors)", ITERATIONS));
    std::cout << " done\n";

    // Mini Cheetah without rotors (URDF)
    std::cout << "  Benchmarking MiniCheetah (no rotors)..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/mini_cheetah_approximate.urdf",
                                    "MiniCheetah (no rotors)", ITERATIONS));
    std::cout << " done\n";

    // MIT Humanoid with rotors
    std::cout << "  Benchmarking MIT_Humanoid (with rotors)..." << std::flush;
    results.push_back(benchmarkRobot<MIT_Humanoid<double, ori_representation::Quaternion>>(
        "MIT_Humanoid (with rotors)", ITERATIONS));
    std::cout << " done\n";

    // MIT Humanoid without rotors
    std::cout << "  Benchmarking MIT_Humanoid (no rotors)..." << std::flush;
    results.push_back(benchmarkRobot<MIT_Humanoid_no_rotors<double, ori_representation::Quaternion>>(
        "MIT_Humanoid (no rotors)", ITERATIONS));
    std::cout << " done\n";

    // ========== Tello Factorial Design: Isolating Rotor Dynamics & Constraint Overhead ==========
    // Factorial design for computation time analysis:
    //   - Factor 1: Rotors (real inertia vs. none)
    //   - Factor 2: Constraints (GenericImplicit/CasADi vs. linear vs. none)
    // This decomposition enables isolation of computational costs.

    // Baseline: no rotors, no constraints (plain tree structure)
    std::cout << "  Benchmarking Tello (-R,-M) [BASELINE]..." << std::flush;
    results.push_back(benchmarkRobot<TelloNoRotors<double>>("Tello (-R,-M) [base]", ITERATIONS));
    std::cout << " done\n";

    // With rotors only (real inertia, no constraint coupling)
    std::cout << "  Benchmarking Tello (+R,-M) [rotor cost]..." << std::flush;
    results.push_back(benchmarkRobot<TelloRotorsNoConstraints<double>>("Tello (+R,-M) [rotors]", ITERATIONS));
    std::cout << " done\n";

    // With linear constraints only (virtual rotors 1e-9 kg, static constraints)
    std::cout << "  Benchmarking Tello (-R,+M-Static) [linear cost]..." << std::flush;
    results.push_back(benchmarkRobot<TelloMechanismsNoRotorsStatic<double>>("Tello (-R/+M-Static) [linear]", ITERATIONS));
    std::cout << " done\n";

    // With CasADi/GenericImplicit constraints only (virtual rotors 1e-9 kg, symbolic differentiation)
    if (envFlagEnabled("GRBDA_BENCH_INCLUDE_TELLO_GENERIC", true)) {
        std::cout << "  Benchmarking Tello (-R,+M-Generic) [CasADi cost]..." << std::flush;
        results.push_back(benchmarkRobot<TelloMechanismsNoRotors<double>>("Tello (-R/+M-Generic) [CasADi]", ITERATIONS));
        std::cout << " done\n";
    }

    // Full model: rotors + CasADi constraints (realistic robot)
    if (envFlagEnabled("GRBDA_BENCH_INCLUDE_TELLO_FULL", true)) {
        std::cout << "  Benchmarking Tello (+R,+M) [FULL MODEL]..." << std::flush;
        results.push_back(benchmarkRobot<Tello<double>>("Tello (+R,+M) [full]", ITERATIONS));
        std::cout << " done\n";
    }

    // Legacy variant for reference (rotors with independent clusters, no constraint coupling)
    std::cout << "  Benchmarking Tello (+R,-M-old) [legacy]..." << std::flush;
    results.push_back(benchmarkRobot<TelloNoMechanisms<double>>("Tello (+R,-M-old) [legacy]", ITERATIONS));
    std::cout << " done\n";

    // Tello with Arms
    if (envFlagEnabled("GRBDA_BENCH_INCLUDE_TELLO_ARMS", true)) {
        std::cout << "  Benchmarking TelloWithArms..." << std::flush;
        results.push_back(benchmarkRobot<TelloWithArms<double>>("TelloWithArms", ITERATIONS));
        std::cout << " done\n";
    }

    // KUKA LWR 4+ (7-DOF serial chain)
    std::cout << "  Benchmarking KUKA LWR 4+..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/kuka_lwr_4plus.urdf",
                                    "KUKA LWR 4+", ITERATIONS));
    std::cout << " done\n";

    // Print results table
    std::cout << "\n" << std::string(75, '=') << "\n";
    std::cout << "First Order ID Derivatives Benchmark Results\n";
    std::cout << "Iterations per robot: " << ITERATIONS << "\n";
    std::cout << std::string(75, '=') << "\n\n";

    std::cout << std::left << std::setw(35) << "Robot"
              << std::right << std::setw(8) << "DOF"
              << std::setw(18) << "Avg Time (us)"
              << std::setw(14) << "Iterations" << "\n";
    std::cout << std::string(75, '-') << "\n";

    for (const auto& r : results) {
        std::cout << std::left << std::setw(35) << r.name
                  << std::right << std::setw(8) << r.dof
                  << std::setw(18) << std::fixed << std::setprecision(2) << r.avg_time_us
                  << std::setw(14) << r.iterations << "\n";
    }

    std::cout << std::string(75, '-') << "\n";

    // Print comparison summary
    std::cout << "\n" << std::string(75, '=') << "\n";
    std::cout << "Speedup Summary (with vs without rotors/mechanisms)\n";
    std::cout << std::string(75, '=') << "\n\n";

    // MiniCheetah comparison
    if (results.size() >= 2) {
        double speedup = results[0].avg_time_us / results[1].avg_time_us;
        std::cout << "MiniCheetah: " << std::fixed << std::setprecision(2)
                  << results[0].avg_time_us << " us (rotors) vs "
                  << results[1].avg_time_us << " us (no rotors) -> "
                  << speedup << "x\n";
    }

    // MIT Humanoid comparison
    if (results.size() >= 4) {
        double speedup = results[2].avg_time_us / results[3].avg_time_us;
        std::cout << "MIT_Humanoid: " << std::fixed << std::setprecision(2)
                  << results[2].avg_time_us << " us (rotors) vs "
                  << results[3].avg_time_us << " us (no rotors) -> "
                  << speedup << "x\n";
    }

    const auto findResult = [&](const std::string& name) -> const BenchmarkResult* {
        for (const auto& r : results) {
            if (r.name == name) {
                return &r;
            }
        }
        return nullptr;
    };

    const auto* tello_full = findResult("Tello (+R,+M) [full]");
    const auto* tello_rotors = findResult("Tello (+R,-M) [rotors]");
    const auto* tello_base = findResult("Tello (-R,-M) [base]");
    const auto* tello_linear = findResult("Tello (-R/+M-Static) [linear]");

    if (tello_full && tello_rotors && tello_base && tello_linear) {
        std::cout << "Tello:\n";
        std::cout << "  +R,+M: " << std::fixed << std::setprecision(2) << tello_full->avg_time_us << " us\n";
        std::cout << "  +R,-M: " << std::fixed << std::setprecision(2) << tello_rotors->avg_time_us << " us\n";
        std::cout << "  -R,-M: " << std::fixed << std::setprecision(2) << tello_base->avg_time_us << " us\n";
        std::cout << "  -R,+M: " << std::fixed << std::setprecision(2) << tello_linear->avg_time_us << " us\n";
        std::cout << "  Mechanisms overhead (with rotors): " << std::fixed << std::setprecision(2)
                  << tello_full->avg_time_us / tello_rotors->avg_time_us << "x (+R,+M vs +R,-M)\n";
        std::cout << "  Mechanisms overhead (no rotors):   " << std::fixed << std::setprecision(2)
                  << tello_linear->avg_time_us / tello_base->avg_time_us << "x (-R,+M vs -R,-M)\n";
        std::cout << "  Rotors overhead (with mechanisms): " << std::fixed << std::setprecision(2)
                  << tello_full->avg_time_us / tello_linear->avg_time_us << "x (+R,+M vs -R,+M)\n";
        std::cout << "  Rotors overhead (no mechanisms):   " << std::fixed << std::setprecision(2)
                  << tello_rotors->avg_time_us / tello_base->avg_time_us << "x (+R,-M vs -R,-M)\n";
        std::cout << "  Total overhead: " << std::fixed << std::setprecision(2)
                  << tello_full->avg_time_us / tello_base->avg_time_us << "x (+R,+M vs -R,-M)\n";
    }

    std::cout << "\n";

    return 0;
}
