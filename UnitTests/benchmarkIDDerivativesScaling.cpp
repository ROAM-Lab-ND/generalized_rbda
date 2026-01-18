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
// This benchmark tests how computational cost and error scale with:
// 1. Number of links in serial chains (RevoluteChainWithRotor)
// 2. Number of links in binary trees (branching kinematic structures)
// 3. Different joint types: simple revolute, revolute pair, revolute triple
// ============================================================================

struct ScalingResult {
    std::string topology;
    std::string joint_type;
    int num_links;
    int dof;
    double avg_time_us;
    double max_error_dq;
    double max_error_dqdot;
};

// Helper to compute finite difference derivatives for validation
template<typename Scalar>
std::pair<DMat<Scalar>, DMat<Scalar>> computeFiniteDifferenceDerivatives(
    ClusterTreeModel<Scalar>& model,
    const DVec<Scalar>& q,
    const DVec<Scalar>& qd,
    const DVec<Scalar>& ydd,
    double h = 1e-7)
{
    const int nDOF = model.getNumDegreesOfFreedom();
    DMat<Scalar> dtau_dq_numerical(nDOF, nDOF);
    DMat<Scalar> dtau_dqdot_numerical(nDOF, nDOF);

    // Numerical dtau/dq
    for (int j = 0; j < nDOF; ++j) {
        DVec<Scalar> q_plus = q;
        q_plus(j) += h;
        DVec<Scalar> q_minus = q;
        q_minus(j) -= h;

        ModelState<Scalar> state_plus;
        int idx = 0;
        for (const auto& cluster : model.clusters()) {
            JointState<Scalar> js;
            js.position = q_plus.segment(idx, cluster->num_positions_);
            js.velocity = qd.segment(idx, cluster->num_velocities_);
            state_plus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model.setState(state_plus);
        DVec<Scalar> tau_plus = model.inverseDynamics(ydd);

        ModelState<Scalar> state_minus;
        idx = 0;
        for (const auto& cluster : model.clusters()) {
            JointState<Scalar> js;
            js.position = q_minus.segment(idx, cluster->num_positions_);
            js.velocity = qd.segment(idx, cluster->num_velocities_);
            state_minus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model.setState(state_minus);
        DVec<Scalar> tau_minus = model.inverseDynamics(ydd);

        dtau_dq_numerical.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    // Numerical dtau/dqdot
    for (int j = 0; j < nDOF; ++j) {
        DVec<Scalar> qd_plus = qd;
        qd_plus(j) += h;
        DVec<Scalar> qd_minus = qd;
        qd_minus(j) -= h;

        ModelState<Scalar> state_plus;
        int idx = 0;
        for (const auto& cluster : model.clusters()) {
            JointState<Scalar> js;
            js.position = q.segment(idx, cluster->num_positions_);
            js.velocity = qd_plus.segment(idx, cluster->num_velocities_);
            state_plus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model.setState(state_plus);
        DVec<Scalar> tau_plus = model.inverseDynamics(ydd);

        ModelState<Scalar> state_minus;
        idx = 0;
        for (const auto& cluster : model.clusters()) {
            JointState<Scalar> js;
            js.position = q.segment(idx, cluster->num_positions_);
            js.velocity = qd_minus.segment(idx, cluster->num_velocities_);
            state_minus.push_back(js);
            idx += cluster->num_velocities_;
        }
        model.setState(state_minus);
        DVec<Scalar> tau_minus = model.inverseDynamics(ydd);

        dtau_dqdot_numerical.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    return {dtau_dq_numerical, dtau_dqdot_numerical};
}

// Generic test function for any robot type
template<typename RobotType>
ScalingResult testScaling(const std::string& topology, const std::string& joint_type, int num_links) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();

    const int nDOF = model.getNumDegreesOfFreedom();

    // Set random state
    ModelState<double> model_state;
    for (const auto& cluster : model.clusters()) {
        model_state.push_back(cluster->joint_->randomJointState());
    }
    model.setState(model_state);

    auto [q, qd] = model.getState();
    DVec<double> ydd = DVec<double>::Random(nDOF);

    // Compute analytical derivatives
    auto [dtau_dq_analytical, dtau_dqdot_analytical] =
        model.firstOrderInverseDynamicsDerivatives(ydd);

    // Compute numerical derivatives
    auto [dtau_dq_numerical, dtau_dqdot_numerical] =
        computeFiniteDifferenceDerivatives(model, q, qd, ydd);

    // Compute errors
    DMat<double> error_dq = dtau_dq_analytical - dtau_dq_numerical;
    DMat<double> error_dqdot = dtau_dqdot_analytical - dtau_dqdot_numerical;

    double max_error_dq = error_dq.cwiseAbs().maxCoeff();
    double max_error_dqdot = error_dqdot.cwiseAbs().maxCoeff();

    // Reset to original state for timing
    model.setState(model_state);

    // Benchmark timing
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

    return {topology, joint_type, num_links, nDOF, avg_time_us, max_error_dq, max_error_dqdot};
}

// Build a binary tree model with N levels (2^N - 1 links total)
// Each node has two children, all joints are revolute
ClusterTreeModel<double> buildBinaryTree(int num_levels) {
    ClusterTreeModel<double> model{};

    Mat3<double> I3 = Mat3<double>::Identity();
    Vec3<double> z3 = Vec3<double>::Zero();

    model.setGravity(Vec3<double>{9.81, 0., 0.});

    // Inertia params
    Mat3<double> link_inertia;
    link_inertia << 0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    const SpatialInertia<double> link_spatial_inertia(1.0, Vec3<double>(0.5, 0., 0.), link_inertia);

    ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

    // Level 0: root node
    std::string root_name = "link_0_0";
    spatial::Transform<double> root_Xtree(I3, z3);
    model.template appendBody<ClusterJoints::Revolute<double>>(
        root_name, link_spatial_inertia, "ground", root_Xtree, axis);

    // Build tree level by level
    std::vector<std::string> current_level = {root_name};

    for (int level = 1; level < num_levels; ++level) {
        std::vector<std::string> next_level;
        int node_idx = 0;

        for (const auto& parent_name : current_level) {
            // Left child
            std::string left_name = "link_" + std::to_string(level) + "_" + std::to_string(node_idx++);
            spatial::Transform<double> left_Xtree(I3, Vec3<double>(1.0, 0., 0.));
            model.template appendBody<ClusterJoints::Revolute<double>>(
                left_name, link_spatial_inertia, parent_name, left_Xtree, axis);
            next_level.push_back(left_name);

            // Right child
            std::string right_name = "link_" + std::to_string(level) + "_" + std::to_string(node_idx++);
            spatial::Transform<double> right_Xtree(I3, Vec3<double>(1.0, 0., 0.));
            model.template appendBody<ClusterJoints::Revolute<double>>(
                right_name, link_spatial_inertia, parent_name, right_Xtree, axis);
            next_level.push_back(right_name);
        }

        current_level = next_level;
    }

    return model;
}

// Test binary tree scaling
ScalingResult testBinaryTreeScaling(int num_levels) {
    ClusterTreeModel<double> model = buildBinaryTree(num_levels);

    const int nDOF = model.getNumDegreesOfFreedom();
    const int num_links = (1 << num_levels) - 1;  // 2^N - 1

    // Set random state
    ModelState<double> model_state;
    for (const auto& cluster : model.clusters()) {
        model_state.push_back(cluster->joint_->randomJointState());
    }
    model.setState(model_state);

    auto [q, qd] = model.getState();
    DVec<double> ydd = DVec<double>::Random(nDOF);

    // Compute analytical derivatives
    auto [dtau_dq_analytical, dtau_dqdot_analytical] =
        model.firstOrderInverseDynamicsDerivatives(ydd);

    // Compute numerical derivatives
    auto [dtau_dq_numerical, dtau_dqdot_numerical] =
        computeFiniteDifferenceDerivatives(model, q, qd, ydd);

    // Compute errors
    DMat<double> error_dq = dtau_dq_analytical - dtau_dq_numerical;
    DMat<double> error_dqdot = dtau_dqdot_analytical - dtau_dqdot_numerical;

    double max_error_dq = error_dq.cwiseAbs().maxCoeff();
    double max_error_dqdot = error_dqdot.cwiseAbs().maxCoeff();

    // Reset to original state for timing
    model.setState(model_state);

    // Benchmark timing
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

    return {"BinaryTree", "Revolute", num_links, nDOF, avg_time_us, max_error_dq, max_error_dqdot};
}

void printResult(const ScalingResult& r) {
    std::cout << std::left << std::setw(14) << r.topology
              << std::setw(16) << r.joint_type
              << std::setw(8) << r.num_links
              << std::setw(6) << r.dof
              << std::setw(14) << std::fixed << std::setprecision(2) << r.avg_time_us
              << std::setw(14) << std::scientific << std::setprecision(2) << r.max_error_dq
              << std::setw(14) << r.max_error_dqdot
              << "\n";
}

void printHeader() {
    std::cout << std::left << std::setw(14) << "Topology"
              << std::setw(16) << "Joint Type"
              << std::setw(8) << "Links"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << std::string(86, '-') << "\n";
}

void analyzeScaling(const std::vector<ScalingResult>& results, const std::string& name) {
    if (results.size() < 2) return;

    // Compute time complexity exponent using first and last results
    double log_ratio_time = std::log(results.back().avg_time_us / results[0].avg_time_us) /
                            std::log(static_cast<double>(results.back().num_links) / results[0].num_links);

    double log_ratio_error = std::log(results.back().max_error_dq / results[0].max_error_dq) /
                             std::log(static_cast<double>(results.back().num_links) / results[0].num_links);

    std::cout << name << " Scaling Analysis:\n";
    std::cout << "  Time complexity: O(n^" << std::fixed << std::setprecision(2) << log_ratio_time << ")\n";
    std::cout << "  Error growth:    O(n^" << std::fixed << std::setprecision(2) << log_ratio_error << ")\n";
    std::cout << "  Time per DOF (first): " << std::fixed << std::setprecision(2)
              << results[0].avg_time_us / results[0].dof << " us/DOF\n";
    std::cout << "  Time per DOF (last):  " << std::fixed << std::setprecision(2)
              << results.back().avg_time_us / results.back().dof << " us/DOF\n\n";
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Inverse Dynamics Derivatives Scaling Benchmark\n";
    std::cout << "===========================================================================\n\n";

    // =========================================================================
    // Test 1: Serial Chain Scaling with Simple Revolute Joints
    // =========================================================================
    std::cout << "Test 1: Serial Chain Scaling - RevoluteChainWithRotor\n";
    printHeader();

    std::vector<ScalingResult> serial_results;

    auto r2 = testScaling<RevoluteChainWithRotor<2, double>>("SerialChain", "RevWithRotor", 2);
    serial_results.push_back(r2);
    printResult(r2);

    auto r4 = testScaling<RevoluteChainWithRotor<4, double>>("SerialChain", "RevWithRotor", 4);
    serial_results.push_back(r4);
    printResult(r4);

    auto r6 = testScaling<RevoluteChainWithRotor<6, double>>("SerialChain", "RevWithRotor", 6);
    serial_results.push_back(r6);
    printResult(r6);

    auto r8 = testScaling<RevoluteChainWithRotor<8, double>>("SerialChain", "RevWithRotor", 8);
    serial_results.push_back(r8);
    printResult(r8);

    auto r10 = testScaling<RevoluteChainWithRotor<10, double>>("SerialChain", "RevWithRotor", 10);
    serial_results.push_back(r10);
    printResult(r10);

    auto r12 = testScaling<RevoluteChainWithRotor<12, double>>("SerialChain", "RevWithRotor", 12);
    serial_results.push_back(r12);
    printResult(r12);

    auto r16 = testScaling<RevoluteChainWithRotor<16, double>>("SerialChain", "RevWithRotor", 16);
    serial_results.push_back(r16);
    printResult(r16);

    auto r20 = testScaling<RevoluteChainWithRotor<20, double>>("SerialChain", "RevWithRotor", 20);
    serial_results.push_back(r20);
    printResult(r20);

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(serial_results, "Serial Chain");

    // =========================================================================
    // Test 2: Binary Tree Scaling
    // =========================================================================
    std::cout << "Test 2: Binary Tree Scaling - Simple Revolute Joints\n";
    printHeader();

    std::vector<ScalingResult> tree_results;

    // 2 levels = 3 links, 3 levels = 7 links, 4 levels = 15 links, 5 levels = 31 links
    for (int levels = 2; levels <= 5; ++levels) {
        auto result = testBinaryTreeScaling(levels);
        tree_results.push_back(result);
        printResult(result);
    }

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(tree_results, "Binary Tree");

    // =========================================================================
    // Test 3: Serial Chain with RevolutePairChainWithRotor (coupled joints)
    // =========================================================================
    std::cout << "Test 3: Serial Chain Scaling - RevolutePairChainWithRotor\n";
    printHeader();

    std::vector<ScalingResult> pair_results;

    auto p2 = testScaling<RevolutePairChainWithRotor<2, double>>("SerialChain", "RevPairWithRotor", 2);
    pair_results.push_back(p2);
    printResult(p2);

    auto p4 = testScaling<RevolutePairChainWithRotor<4, double>>("SerialChain", "RevPairWithRotor", 4);
    pair_results.push_back(p4);
    printResult(p4);

    auto p6 = testScaling<RevolutePairChainWithRotor<6, double>>("SerialChain", "RevPairWithRotor", 6);
    pair_results.push_back(p6);
    printResult(p6);

    auto p8 = testScaling<RevolutePairChainWithRotor<8, double>>("SerialChain", "RevPairWithRotor", 8);
    pair_results.push_back(p8);
    printResult(p8);

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(pair_results, "RevolutePair Chain");

    // =========================================================================
    // Test 4: Serial Chain with RevoluteTripleChainWithRotor (triple-coupled joints)
    // =========================================================================
    std::cout << "Test 4: Serial Chain Scaling - RevoluteTripleChainWithRotor\n";
    printHeader();

    std::vector<ScalingResult> triple_results;

    auto t3 = testScaling<RevoluteTripleChainWithRotor<3, double>>("SerialChain", "RevTripleWithRotor", 3);
    triple_results.push_back(t3);
    printResult(t3);

    auto t6 = testScaling<RevoluteTripleChainWithRotor<6, double>>("SerialChain", "RevTripleWithRotor", 6);
    triple_results.push_back(t6);
    printResult(t6);

    auto t9 = testScaling<RevoluteTripleChainWithRotor<9, double>>("SerialChain", "RevTripleWithRotor", 9);
    triple_results.push_back(t9);
    printResult(t9);

    auto t12 = testScaling<RevoluteTripleChainWithRotor<12, double>>("SerialChain", "RevTripleWithRotor", 12);
    triple_results.push_back(t12);
    printResult(t12);

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(triple_results, "RevoluteTriple Chain");

    // =========================================================================
    // Summary Comparison
    // =========================================================================
    std::cout << "===========================================================================\n";
    std::cout << "Summary: Comparison at Similar DOF Counts\n";
    std::cout << "===========================================================================\n\n";

    std::cout << "~6 DOF Systems:\n";
    std::cout << "  Serial RevWithRotor (6 links): " << std::fixed << std::setprecision(2)
              << r6.avg_time_us << " us\n";
    std::cout << "  RevPairWithRotor (6 links):    " << std::fixed << std::setprecision(2)
              << p6.avg_time_us << " us\n";
    std::cout << "  RevTripleWithRotor (6 links):  " << std::fixed << std::setprecision(2)
              << t6.avg_time_us << " us\n";
    std::cout << "  Binary Tree (7 links):         " << std::fixed << std::setprecision(2)
              << tree_results[1].avg_time_us << " us\n\n";

    std::cout << "Observations:\n";
    std::cout << "1. Serial chains should show O(n) to O(n^2) scaling depending on algorithm.\n";
    std::cout << "2. Binary trees may show different scaling due to parallel branches.\n";
    std::cout << "3. Complex joints (pair, triple) add overhead for constraint handling.\n";
    std::cout << "4. Error should remain bounded regardless of system size.\n\n";

    std::cout << "===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n";

    return 0;
}
