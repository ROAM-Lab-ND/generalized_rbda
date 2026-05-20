/**
 * @file benchmarkIDDerivativesBreakdown.cpp
 * @brief Benchmark ID derivatives with detailed profiling breakdown for figure generation.
 *
 * Outputs CSV data suitable for plotting performance breakdown figures.
 */

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

struct ProfilingResult {
    std::string robot_name;     // Internal name for CSV
    std::string label;          // Display label
    int dof;
    int bodies;
    double fwd_kin_us;
    double fwd_casadi_us;
    double fwd_other_us;
    double bwd_casadi_us;
    double bwd_other_us;
    double bwd_prop_us;
    double total_us;
};

ProfilingResult profileModel(ClusterTreeModel<double>& model,
                              const std::string& robot_name,
                              const std::string& label,
                              int iterations = 1000) {
    const int nDOF = model.getNumDegreesOfFreedom();
    const int bodies = model.getNumBodies();

    // Set random state
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

    // Profile iterations
    enableIDDerivativesProfiling();
    for (int i = 0; i < iterations; ++i) {
        auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
        (void)dtau_dq;
        (void)dtau_dqdot;
    }

    auto data = getIDDerivativesProfilingData();
    resetIDDerivativesProfiling();

    return {
        robot_name,
        label,
        nDOF,
        bodies,
        data[0],  // fwd_kin
        data[1],  // fwd_casadi
        data[2],  // fwd_other
        data[3],  // bwd_casadi
        data[4],  // bwd_other
        data[5],  // bwd_prop
        data[6]   // total
    };
}

template<typename RobotType>
ProfilingResult profileRobot(const std::string& robot_name,
                              const std::string& label,
                              int iterations = 1000) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    return profileModel(model, robot_name, label, iterations);
}

ProfilingResult profileURDF(const std::string& urdf_path,
                             const std::string& robot_name,
                             const std::string& label,
                             int iterations = 1000) {
    ClusterTreeModel<double> model;
    model.buildModelFromURDF(urdf_path);
    return profileModel(model, robot_name, label, iterations);
}

int main(int argc, char** argv) {
    std::vector<ProfilingResult> results;
    const int ITERATIONS = 1000;
    const std::string urdf_path = std::string(SOURCE_DIRECTORY) + "/robot-models";

    std::cout << "\nProfiling ID derivatives breakdown...\n\n";

    // KUKA LWR 4+
    std::cout << "  KUKA LWR 4+..." << std::flush;
    results.push_back(profileURDF(urdf_path + "/kuka_lwr_4plus.urdf",
                                   "KUKA_LWR_4plus", "KUKA LWR 4+ (-R)", ITERATIONS));
    std::cout << " done\n";

    // MiniCheetah with rotors
    std::cout << "  MiniCheetah (+R)..." << std::flush;
    results.push_back(profileRobot<MiniCheetah<double, ori_representation::Quaternion>>(
        "MiniCheetah_rotors", "Mini Cheetah (+R)", ITERATIONS));
    std::cout << " done\n";

    // MiniCheetah without rotors
    std::cout << "  MiniCheetah (-R)..." << std::flush;
    results.push_back(profileURDF(urdf_path + "/mini_cheetah_approximate.urdf",
                                   "MiniCheetah_no_rotors", "Mini Cheetah (-R)", ITERATIONS));
    std::cout << " done\n";

    // MIT Humanoid with rotors
    std::cout << "  MIT_Humanoid (+R)..." << std::flush;
    results.push_back(profileRobot<MIT_Humanoid<double, ori_representation::Quaternion>>(
        "MIT_Humanoid_rotors", "MIT Humanoid (+R)", ITERATIONS));
    std::cout << " done\n";

    // MIT Humanoid without rotors
    std::cout << "  MIT_Humanoid (-R)..." << std::flush;
    results.push_back(profileRobot<MIT_Humanoid_no_rotors<double, ori_representation::Quaternion>>(
        "MIT_Humanoid_no_rotors", "MIT Humanoid (-R)", ITERATIONS));
    std::cout << " done\n";

    // Tello (-R/-M) - no rotors, no mechanisms (baseline)
    std::cout << "  Tello (-R/-M)..." << std::flush;
    results.push_back(profileRobot<TelloNoRotors<double>>(
        "Tello_no_rotors_no_mech", "Tello (-R/-M)", ITERATIONS));
    std::cout << " done\n";

    // Tello (+R/-M) - rotors, no mechanisms
    std::cout << "  Tello (+R/-M)..." << std::flush;
    results.push_back(profileRobot<TelloRotorsNoConstraints<double>>(
        "Tello_rotors_no_mech", "Tello (+R/-M)", ITERATIONS));
    std::cout << " done\n";

    // Tello (+R/+M) - rotors with mechanisms (CasADi)
    std::cout << "  Tello (+R/+M)..." << std::flush;
    results.push_back(profileRobot<Tello<double>>(
        "Tello_rotors_mech", "Tello (+R/+M)", ITERATIONS));
    std::cout << " done\n";

    // TelloWithArms
    std::cout << "  TelloWithArms..." << std::flush;
    results.push_back(profileRobot<TelloWithArms<double>>(
        "TelloWithArms", "Tello with Arms (+R/+M)", ITERATIONS));
    std::cout << " done\n";

    // Cassie (closed-loop biped)
    std::cout << "  Cassie (closed-loop)..." << std::flush;
    results.push_back(profileRobot<Cassie<double>>(
        "Cassie", "Cassie (closed-loop)", ITERATIONS));
    std::cout << " done\n";

    // Print results table
    std::cout << "\n" << std::string(120, '=') << "\n";
    std::cout << "ID Derivatives Profiling Breakdown (us/call)\n";
    std::cout << std::string(120, '=') << "\n\n";

    std::cout << std::left << std::setw(28) << "Robot"
              << std::right << std::setw(6) << "DOF"
              << std::setw(10) << "FwdKin"
              << std::setw(10) << "FwdCasADi"
              << std::setw(10) << "FwdOther"
              << std::setw(10) << "BwdCasADi"
              << std::setw(10) << "BwdOther"
              << std::setw(10) << "BwdProp"
              << std::setw(10) << "Total" << "\n";
    std::cout << std::string(120, '-') << "\n";

    for (const auto& r : results) {
        std::cout << std::left << std::setw(28) << r.label
                  << std::right << std::setw(6) << r.dof
                  << std::setw(10) << std::fixed << std::setprecision(2) << r.fwd_kin_us
                  << std::setw(10) << r.fwd_casadi_us
                  << std::setw(10) << r.fwd_other_us
                  << std::setw(10) << r.bwd_casadi_us
                  << std::setw(10) << r.bwd_other_us
                  << std::setw(10) << r.bwd_prop_us
                  << std::setw(10) << r.total_us << "\n";
    }
    std::cout << std::string(120, '-') << "\n";

    // Write CSV
    std::string csv_path = std::string(SOURCE_DIRECTORY) + "/Benchmarking/data/fig4_performance_breakdown_current.csv";
    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "robot_name,label,dof,bodies,fwd_kin_us,fwd_casadi_us,fwd_other_us,bwd_casadi_us,bwd_other_us,bwd_prop_us,total_us\n";
        for (const auto& r : results) {
            csv << r.robot_name << ","
                << r.label << ","
                << r.dof << ","
                << r.bodies << ","
                << std::fixed << std::setprecision(4)
                << r.fwd_kin_us << ","
                << r.fwd_casadi_us << ","
                << r.fwd_other_us << ","
                << r.bwd_casadi_us << ","
                << r.bwd_other_us << ","
                << r.bwd_prop_us << ","
                << r.total_us << "\n";
        }
        csv.close();
        std::cout << "\nCSV written to: " << csv_path << "\n";
    } else {
        std::cerr << "\nWarning: Could not write CSV to " << csv_path << "\n";
    }

    return 0;
}
