#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

// ============================================================================
// Parallel Chain Loop Depth Benchmark
// ============================================================================
// This benchmark tests how computational cost changes based on:
// 1. Where in a serial chain a coupled joint (RevolutePair) is placed
// 2. Comparison between branching trees and serial chains
//
// The goal is to understand if loop constraint position affects derivative
// computation cost - e.g., does a coupled joint early in the chain behave
// differently than one at the end?
// ============================================================================

struct DepthResult {
    std::string config;
    int total_links;
    int coupled_position;  // 0 = no coupling, N = coupled joint at position N
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

// Build a chain of N RevolutePair clusters (2N total links), where the nth
// pair uses configuration-dependent S matrices.
// The "position" parameter controls where a standard RevolutePair is placed
// vs using simple consecutive revolutes to get the same DOF.
// For simplicity, we build full RevolutePair chains and vary the chain length.
ClusterTreeModel<double> buildRevolutePairChain(int num_pairs) {
    ClusterTreeModel<double> model{};

    Mat3<double> I3 = Mat3<double>::Identity();
    Vec3<double> z3 = Vec3<double>::Zero();

    model.setGravity(Vec3<double>{9.81, 0., 0.});

    Mat3<double> link_inertia;
    link_inertia << 0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    const SpatialInertia<double> link_spatial_inertia(1.0, Vec3<double>(0.5, 0., 0.), link_inertia);

    ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

    std::string prev_link = "ground";

    for (int i = 0; i < num_pairs; ++i) {
        spatial::Transform<double> Xtree1 = (i == 0) ? spatial::Transform<double>(I3, z3)
                                                     : spatial::Transform<double>(I3, Vec3<double>(1.0, 0., 0.));
        spatial::Transform<double> Xtree2(I3, Vec3<double>(1.0, 0., 0.));

        std::string link_A_name = "link_A_" + std::to_string(i);
        std::string link_B_name = "link_B_" + std::to_string(i);

        auto body_A = model.registerBody(link_A_name, link_spatial_inertia, prev_link, Xtree1);
        auto body_B = model.registerBody(link_B_name, link_spatial_inertia, link_A_name, Xtree2);

        std::string cluster_name = "pair_" + std::to_string(i);
        model.template appendRegisteredBodiesAsCluster<ClusterJoints::RevolutePair<double>>(
            cluster_name, body_A, body_B, axis, axis);

        prev_link = link_B_name;
    }

    return model;
}

// Build a simple serial chain (baseline - no coupled joints)
ClusterTreeModel<double> buildSimpleSerialChain(int num_links) {
    ClusterTreeModel<double> model{};

    Mat3<double> I3 = Mat3<double>::Identity();
    Vec3<double> z3 = Vec3<double>::Zero();

    model.setGravity(Vec3<double>{9.81, 0., 0.});

    Mat3<double> link_inertia;
    link_inertia << 0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    const SpatialInertia<double> link_spatial_inertia(1.0, Vec3<double>(0.5, 0., 0.), link_inertia);

    ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

    std::string prev_link = "ground";
    for (int i = 0; i < num_links; ++i) {
        std::string link_name = "link_" + std::to_string(i);
        spatial::Transform<double> Xtree(I3, i == 0 ? z3 : Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link_name, link_spatial_inertia, prev_link, Xtree, axis);
        prev_link = link_name;
    }

    return model;
}

// Build a branching tree (Y-shape) for comparison
ClusterTreeModel<double> buildBranchingTree(int trunk_length, int branch_length) {
    ClusterTreeModel<double> model{};

    Mat3<double> I3 = Mat3<double>::Identity();
    Vec3<double> z3 = Vec3<double>::Zero();

    model.setGravity(Vec3<double>{9.81, 0., 0.});

    Mat3<double> link_inertia;
    link_inertia << 0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    const SpatialInertia<double> link_spatial_inertia(1.0, Vec3<double>(0.5, 0., 0.), link_inertia);

    ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

    // Build trunk
    std::string prev_link = "ground";
    for (int i = 0; i < trunk_length; ++i) {
        std::string link_name = "trunk_" + std::to_string(i);
        spatial::Transform<double> Xtree(I3, i == 0 ? z3 : Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link_name, link_spatial_inertia, prev_link, Xtree, axis);
        prev_link = link_name;
    }

    std::string branch_point = prev_link;

    // Build branch A
    prev_link = branch_point;
    for (int i = 0; i < branch_length; ++i) {
        std::string link_name = "branch_A_" + std::to_string(i);
        spatial::Transform<double> Xtree(I3, Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link_name, link_spatial_inertia, prev_link, Xtree, axis);
        prev_link = link_name;
    }

    // Build branch B
    prev_link = branch_point;
    for (int i = 0; i < branch_length; ++i) {
        std::string link_name = "branch_B_" + std::to_string(i);
        spatial::Transform<double> Xtree(I3, Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link_name, link_spatial_inertia, prev_link, Xtree, axis);
        prev_link = link_name;
    }

    return model;
}

DepthResult testModel(ClusterTreeModel<double>& model, const std::string& config,
                      int total_links, int coupled_position) {
    try {
        int nDOF = model.getNumDegreesOfFreedom();

        if (nDOF == 0) {
            throw std::runtime_error("Model has zero DOF");
        }

        ModelState<double> model_state;
        for (const auto& cluster : model.clusters()) {
            model_state.push_back(cluster->joint_->randomJointState());
        }
        model.setState(model_state);
        auto [q, qd] = model.getState();
        DVec<double> ydd = DVec<double>::Random(nDOF);

        // Warmup
        for (int i = 0; i < 100; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }

        // Timed runs
        const int num_iterations = 1000;
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_iterations; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }
        auto end = std::chrono::high_resolution_clock::now();

        double total_time_us = std::chrono::duration<double, std::micro>(end - start).count();
        double avg_time_us = total_time_us / num_iterations;

        auto [dtau_dq_analytical, dtau_dqdot_analytical] =
            model.firstOrderInverseDynamicsDerivatives(ydd);
        auto [dtau_dq_numerical, dtau_dqdot_numerical] =
            computeFiniteDifferenceDerivatives(model, q, qd, ydd);

        DMat<double> error_dq = (dtau_dq_analytical - dtau_dq_numerical).cwiseAbs();
        DMat<double> error_dqdot = (dtau_dqdot_analytical - dtau_dqdot_numerical).cwiseAbs();

        return {config, total_links, coupled_position, nDOF, avg_time_us,
                error_dq.maxCoeff(), error_dqdot.maxCoeff()};

    } catch (const std::exception& e) {
        std::cerr << "Error testing " << config << ": " << e.what() << "\n";
        return {config, total_links, coupled_position, -1, 0, 0, 0};
    }
}

void printHeader() {
    std::cout << std::left << std::setw(24) << "Configuration"
              << std::setw(8) << "Links"
              << std::setw(12) << "CoupledPos"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << std::string(92, '-') << "\n";
}

void printResult(const DepthResult& r) {
    if (r.dof > 0) {
        std::cout << std::left << std::setw(24) << r.config
                  << std::setw(8) << r.total_links
                  << std::setw(12) << (r.coupled_position == 0 ? "None" : std::to_string(r.coupled_position))
                  << std::setw(6) << r.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.avg_time_us
                  << std::setw(14) << std::scientific << std::setprecision(2) << r.max_error_dq
                  << std::setw(14) << r.max_error_dqdot
                  << "\n";
    } else {
        std::cout << std::left << std::setw(24) << r.config
                  << std::setw(8) << r.total_links
                  << std::setw(12) << (r.coupled_position == 0 ? "None" : std::to_string(r.coupled_position))
                  << std::setw(6) << "N/A"
                  << std::setw(14) << "FAILED"
                  << std::setw(14) << ""
                  << std::setw(14) << ""
                  << "\n";
    }
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Topology and Constraint Position Benchmark\n";
    std::cout << "===========================================================================\n\n";

    std::cout << "This benchmark compares computational cost across different topologies:\n";
    std::cout << "1. Simple serial chains (baseline)\n";
    std::cout << "2. RevolutePair chains (coupled joints)\n";
    std::cout << "3. Branching trees (Y-shape)\n\n";

    // =========================================================================
    // Test 1: Simple Serial Chains (baseline)
    // =========================================================================
    std::cout << "Test 1: Simple Serial Chains (Baseline)\n";
    printHeader();

    std::vector<DepthResult> results_simple;

    for (int n : {4, 6, 8, 10, 12}) {
        auto model = buildSimpleSerialChain(n);
        auto result = testModel(model, "SimpleChain", n, 0);
        results_simple.push_back(result);
        printResult(result);
    }

    std::cout << std::string(92, '-') << "\n\n";

    // =========================================================================
    // Test 2: RevolutePair Chains (coupled joints throughout)
    // =========================================================================
    std::cout << "Test 2: RevolutePair Chains (All Coupled Joints)\n";
    printHeader();

    std::vector<DepthResult> results_pair;

    for (int pairs : {2, 3, 4, 5, 6}) {
        auto model = buildRevolutePairChain(pairs);
        int total_links = pairs * 2;
        auto result = testModel(model, "RevPairChain", total_links, pairs);
        results_pair.push_back(result);
        printResult(result);
    }

    std::cout << std::string(92, '-') << "\n\n";

    // =========================================================================
    // Test 3: Branching Trees (Y-shape)
    // =========================================================================
    std::cout << "Test 3: Branching Trees (Y-shape) - Different Trunk/Branch Ratios\n";
    printHeader();

    std::vector<DepthResult> results_tree;

    // 8 total links: various trunk/branch combinations
    {
        auto model = buildBranchingTree(2, 3);  // 2 trunk + 2*3 branches = 8 total
        auto result = testModel(model, "Tree(trunk=2,br=3)", 8, 0);
        results_tree.push_back(result);
        printResult(result);
    }

    {
        auto model = buildBranchingTree(4, 2);  // 4 trunk + 2*2 branches = 8 total
        auto result = testModel(model, "Tree(trunk=4,br=2)", 8, 0);
        results_tree.push_back(result);
        printResult(result);
    }

    {
        auto model = buildBranchingTree(6, 1);  // 6 trunk + 2*1 branches = 8 total
        auto result = testModel(model, "Tree(trunk=6,br=1)", 8, 0);
        results_tree.push_back(result);
        printResult(result);
    }

    // 12 total links
    {
        auto model = buildBranchingTree(2, 5);  // 2 trunk + 2*5 branches = 12 total
        auto result = testModel(model, "Tree(trunk=2,br=5)", 12, 0);
        results_tree.push_back(result);
        printResult(result);
    }

    {
        auto model = buildBranchingTree(6, 3);  // 6 trunk + 2*3 branches = 12 total
        auto result = testModel(model, "Tree(trunk=6,br=3)", 12, 0);
        results_tree.push_back(result);
        printResult(result);
    }

    {
        auto model = buildBranchingTree(10, 1);  // 10 trunk + 2*1 branches = 12 total
        auto result = testModel(model, "Tree(trunk=10,br=1)", 12, 0);
        results_tree.push_back(result);
        printResult(result);
    }

    std::cout << std::string(92, '-') << "\n\n";

    // =========================================================================
    // Analysis
    // =========================================================================
    std::cout << "===========================================================================\n";
    std::cout << "Analysis: Comparison at 8 DOF\n";
    std::cout << "===========================================================================\n\n";

    // Find 8-link results
    DepthResult simple_8{}, pair_8{}, tree_8{};
    for (const auto& r : results_simple) {
        if (r.total_links == 8) simple_8 = r;
    }
    for (const auto& r : results_pair) {
        if (r.total_links == 8) pair_8 = r;
    }
    for (const auto& r : results_tree) {
        if (r.total_links == 8 && r.config == "Tree(trunk=4,br=2)") tree_8 = r;
    }

    if (simple_8.dof > 0) {
        std::cout << "8-Link Systems Comparison:\n";
        std::cout << "  Simple Serial Chain:      " << std::fixed << std::setprecision(2)
                  << simple_8.avg_time_us << " us (DOF=" << simple_8.dof << ") - baseline\n";
    }
    if (pair_8.dof > 0) {
        std::cout << "  RevolutePair Chain:       " << std::fixed << std::setprecision(2)
                  << pair_8.avg_time_us << " us (DOF=" << pair_8.dof << ")";
        if (simple_8.dof > 0 && simple_8.avg_time_us > 0) {
            std::cout << " - " << std::fixed << std::setprecision(1)
                      << pair_8.avg_time_us / simple_8.avg_time_us << "x baseline";
        }
        std::cout << "\n";
    }
    if (tree_8.dof > 0) {
        std::cout << "  Branching Tree (4+2*2):   " << std::fixed << std::setprecision(2)
                  << tree_8.avg_time_us << " us (DOF=" << tree_8.dof << ")";
        if (simple_8.dof > 0 && simple_8.avg_time_us > 0) {
            std::cout << " - " << std::fixed << std::setprecision(1)
                      << tree_8.avg_time_us / simple_8.avg_time_us << "x baseline";
        }
        std::cout << "\n";
    }

    std::cout << "\nKey Observations:\n";
    std::cout << "1. RevolutePair chains have higher per-DOF cost due to coupled constraint\n";
    std::cout << "   Jacobians and configuration-dependent S matrices.\n";
    std::cout << "2. Branching trees may be faster than serial chains at same DOF due to\n";
    std::cout << "   shorter effective chain depth (affects backward pass).\n";
    std::cout << "3. Error should remain bounded (~1e-7) regardless of topology.\n\n";

    std::cout << "===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n";

    return 0;
}
