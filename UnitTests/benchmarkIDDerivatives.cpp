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

    // Tello with mechanisms
    std::cout << "  Benchmarking Tello (with mechanisms)..." << std::flush;
    results.push_back(benchmarkRobot<Tello<double>>("Tello (with mechanisms)", ITERATIONS));
    std::cout << " done\n";

    // Tello without mechanisms (URDF)
    std::cout << "  Benchmarking Tello (no mechanisms)..." << std::flush;
    results.push_back(benchmarkURDF(urdf_path + "/tello_humanoid_approximate.urdf",
                                    "Tello (no mechanisms)", ITERATIONS));
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

    // Tello comparison
    if (results.size() >= 6) {
        double speedup = results[4].avg_time_us / results[5].avg_time_us;
        std::cout << "Tello: " << std::fixed << std::setprecision(2)
                  << results[4].avg_time_us << " us (mechanisms) vs "
                  << results[5].avg_time_us << " us (no mechanisms) -> "
                  << speedup << "x\n";
    }

    std::cout << "\n";

    return 0;
}
