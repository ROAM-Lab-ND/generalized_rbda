#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

struct BenchmarkResult {
    std::string name;
    int dof;
    double avg_time_us;
    int iterations;
};

BenchmarkResult benchmarkModel(ClusterTreeModel<double>& model, const std::string& name, int iterations) {
    const int nDOF = model.getNumDegreesOfFreedom();

    // Set random state
    ModelState<double> model_state;
    for (const auto& cluster : model.clusters()) {
        model_state.push_back(cluster->joint_->randomJointState());
    }
    model.setState(model_state);

    DVec<double> ydd = DVec<double>::Random(nDOF);

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

    // Full model: rotors + CasADi constraints (realistic robot)
    std::cout << "  Benchmarking Tello (+R,+M) [FULL MODEL]..." << std::flush;
    results.push_back(benchmarkRobot<Tello<double>>("Tello (+R,+M) [full]", ITERATIONS));
    std::cout << " done\n";

    // Profiling breakdown for Tello (full model)
    std::cout << "\n  Running Tello profiling breakdown..." << std::flush;
    {
        Tello<double> robot;
        ClusterTreeModel<double> model = robot.buildClusterTreeModel();
        const int nDOF = model.getNumDegreesOfFreedom();

        ModelState<double> model_state;
        for (const auto& cluster : model.clusters()) {
            model_state.push_back(cluster->joint_->randomJointState());
        }
        model.setState(model_state);

        DVec<double> ydd = DVec<double>::Random(nDOF);

        // Warmup (100 calls)
        for (int i = 0; i < 100; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }

        // Enable profiling and run 1000 iterations
        enableIDDerivativesProfiling();
        for (int i = 0; i < 1000; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }
        printIDDerivativesProfiling();
    }
    std::cout << " done\n";

    // Tello with Arms
    std::cout << "  Benchmarking TelloWithArms..." << std::flush;
    results.push_back(benchmarkRobot<TelloWithArms<double>>("TelloWithArms", ITERATIONS));
    std::cout << " done\n";

    // KUKA LWR 4+ (7-DOF serial chain)
    std::cout << "  Benchmarking KUKA LWR 4+..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/kuka_lwr_4plus.urdf",
                                    "KUKA LWR 4+", ITERATIONS));
    std::cout << " done\n";

    // ========== Closed-Loop Humanoid Robots ==========

    // Cassie (closed-loop leg)
    std::cout << "  Benchmarking Cassie (closed-loop)..." << std::flush;
    results.push_back(benchmarkRobot<Cassie<double>>("Cassie (closed-loop)", ITERATIONS));
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

    // Tello comparison (3 variants at indices 4, 5, 6)
    // 4: -R,-M (TelloNoRotors) baseline
    // 5: +R,-M (TelloRotorsNoConstraints)
    // 6: +R,+M (full Tello)
    if (results.size() >= 7) {
        std::cout << "Tello:\n";
        std::cout << "  -R,-M: " << std::fixed << std::setprecision(2) << results[4].avg_time_us << " us\n";
        std::cout << "  +R,-M: " << std::fixed << std::setprecision(2) << results[5].avg_time_us << " us\n";
        std::cout << "  +R,+M: " << std::fixed << std::setprecision(2) << results[6].avg_time_us << " us\n";
        std::cout << "  Rotor overhead:      " << std::fixed << std::setprecision(2)
                  << results[5].avg_time_us / results[4].avg_time_us << "x (+R,-M vs -R,-M)\n";
        std::cout << "  Mechanism overhead:  " << std::fixed << std::setprecision(2)
                  << results[6].avg_time_us / results[5].avg_time_us << "x (+R,+M vs +R,-M)\n";
        std::cout << "  Total overhead:      " << std::fixed << std::setprecision(2)
                  << results[6].avg_time_us / results[4].avg_time_us << "x (+R,+M vs -R,-M)\n";
    }

    std::cout << "\n";

    // Export results to CSV
    std::string csv_path = std::string(SOURCE_DIRECTORY) + "/../benchmark_figures/data/robot_performance.csv";
    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "robot_name,label,dof,time_us\n";
        for (const auto& r : results) {
            // Create a clean CSV name from the display name
            std::string csv_name = r.name;
            // Replace spaces and special chars for CSV compatibility
            for (char& c : csv_name) {
                if (c == ' ' || c == '(' || c == ')' || c == '/' || c == ',') c = '_';
            }
            csv << csv_name << ","
                << r.name << ","
                << r.dof << ","
                << std::fixed << std::setprecision(2) << r.avg_time_us << "\n";
        }
        csv.close();
        std::cout << "CSV written to: " << csv_path << "\n";
    } else {
        std::cerr << "Warning: Could not write CSV to " << csv_path << "\n";
    }

    return 0;
}
