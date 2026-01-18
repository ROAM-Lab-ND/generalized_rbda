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
// Complex Joint Chain Scaling Benchmark
// ============================================================================
// This benchmark compares scaling behavior across different joint types:
// 1. RevoluteChainWithRotor - simple revolute joints with rotors (baseline)
// 2. RevolutePairChainWithRotor - coupled pairs of revolute joints
// 3. RevoluteTripleChainWithRotor - triple-coupled revolute joints
//
// For each joint type, we test chains of increasing length to understand
// how the joint complexity affects computational scaling.
// ============================================================================

struct ChainResult {
    std::string joint_type;
    int num_links;
    int num_clusters;
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

template <typename ChainType>
ChainResult testChain(const std::string& joint_type, int num_links) {
    try {
        ChainType chain;
        ClusterTreeModel<double> model = chain.buildClusterTreeModel();
        int nDOF = model.getNumDegreesOfFreedom();
        int num_clusters = static_cast<int>(model.clusters().size());

        if (nDOF == 0) {
            throw std::runtime_error("Model has zero DOF");
        }

        // Set random state
        ModelState<double> model_state;
        for (const auto& cluster : model.clusters()) {
            model_state.push_back(cluster->joint_->randomJointState());
        }
        model.setState(model_state);
        auto [q, qd] = model.getState();
        DVec<double> ydd = DVec<double>::Random(nDOF);

        // Warmup
        for (int i = 0; i < 100; ++i) {
            auto [dtau_dq, dtau_dqdot] =
                model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }

        // Timed runs
        const int num_iterations = 1000;
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_iterations; ++i) {
            auto [dtau_dq, dtau_dqdot] =
                model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }
        auto end = std::chrono::high_resolution_clock::now();

        double total_time_us = std::chrono::duration<double, std::micro>(end - start).count();
        double avg_time_us = total_time_us / num_iterations;

        // Compute analytical derivatives for error checking
        auto [dtau_dq_analytical, dtau_dqdot_analytical] =
            model.firstOrderInverseDynamicsDerivatives(ydd);

        // Compute numerical derivatives
        auto [dtau_dq_numerical, dtau_dqdot_numerical] =
            computeFiniteDifferenceDerivatives(model, q, qd, ydd);

        // Compute errors
        DMat<double> error_dq = (dtau_dq_analytical - dtau_dq_numerical).cwiseAbs();
        DMat<double> error_dqdot = (dtau_dqdot_analytical - dtau_dqdot_numerical).cwiseAbs();

        return {joint_type, num_links, num_clusters, nDOF, avg_time_us,
                error_dq.maxCoeff(), error_dqdot.maxCoeff()};

    } catch (const std::exception& e) {
        std::cerr << "Error testing " << joint_type << " with " << num_links << " links: " << e.what() << "\n";
        return {joint_type + " (ERROR)", num_links, 0, -1, 0, 0, 0};
    }
}

void printHeader() {
    std::cout << std::left << std::setw(20) << "Joint Type"
              << std::setw(8) << "Links"
              << std::setw(10) << "Clusters"
              << std::setw(6) << "DOF"
              << std::setw(14) << "Time (us)"
              << std::setw(14) << "Max Err dq"
              << std::setw(14) << "Max Err dqd"
              << "\n";
    std::cout << std::string(86, '-') << "\n";
}

void printResult(const ChainResult& r) {
    if (r.dof > 0) {
        std::cout << std::left << std::setw(20) << r.joint_type
                  << std::setw(8) << r.num_links
                  << std::setw(10) << r.num_clusters
                  << std::setw(6) << r.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.avg_time_us
                  << std::setw(14) << std::scientific << std::setprecision(2) << r.max_error_dq
                  << std::setw(14) << r.max_error_dqdot
                  << "\n";
    } else {
        std::cout << std::left << std::setw(20) << r.joint_type
                  << std::setw(8) << r.num_links
                  << std::setw(10) << "N/A"
                  << std::setw(6) << "N/A"
                  << std::setw(14) << "FAILED"
                  << std::setw(14) << ""
                  << std::setw(14) << ""
                  << "\n";
    }
}

void analyzeScaling(const std::vector<ChainResult>& results, const std::string& name) {
    // Filter out failed results
    std::vector<ChainResult> valid_results;
    for (const auto& r : results) {
        if (r.dof > 0) valid_results.push_back(r);
    }

    if (valid_results.size() < 2) return;

    double log_ratio_time = std::log(valid_results.back().avg_time_us / valid_results[0].avg_time_us) /
                            std::log(static_cast<double>(valid_results.back().num_links) / valid_results[0].num_links);

    std::cout << name << " Scaling:\n";
    std::cout << "  Time complexity: O(n^" << std::fixed << std::setprecision(2) << log_ratio_time << ")\n";
    std::cout << "  Time per DOF (smallest): " << std::fixed << std::setprecision(2)
              << valid_results[0].avg_time_us / valid_results[0].dof << " us/DOF\n";
    std::cout << "  Time per DOF (largest):  " << std::fixed << std::setprecision(2)
              << valid_results.back().avg_time_us / valid_results.back().dof << " us/DOF\n\n";
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Complex Joint Chain Scaling Benchmark\n";
    std::cout << "===========================================================================\n\n";

    // =========================================================================
    // Test 1: RevoluteChainWithRotor (Baseline)
    // =========================================================================
    std::cout << "Test 1: RevoluteChainWithRotor - Baseline (1 DOF per cluster)\n";
    printHeader();

    std::vector<ChainResult> simple_results;

    auto s2 = testChain<RevoluteChainWithRotor<2, double>>("RevWithRotor", 2);
    simple_results.push_back(s2);
    printResult(s2);

    auto s4 = testChain<RevoluteChainWithRotor<4, double>>("RevWithRotor", 4);
    simple_results.push_back(s4);
    printResult(s4);

    auto s6 = testChain<RevoluteChainWithRotor<6, double>>("RevWithRotor", 6);
    simple_results.push_back(s6);
    printResult(s6);

    auto s8 = testChain<RevoluteChainWithRotor<8, double>>("RevWithRotor", 8);
    simple_results.push_back(s8);
    printResult(s8);

    auto s10 = testChain<RevoluteChainWithRotor<10, double>>("RevWithRotor", 10);
    simple_results.push_back(s10);
    printResult(s10);

    auto s12 = testChain<RevoluteChainWithRotor<12, double>>("RevWithRotor", 12);
    simple_results.push_back(s12);
    printResult(s12);

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(simple_results, "RevoluteChainWithRotor");

    // =========================================================================
    // Test 2: RevolutePairChainWithRotor (2 DOF per cluster)
    // =========================================================================
    std::cout << "Test 2: RevolutePairChainWithRotor - Coupled Pairs (2 DOF per cluster)\n";
    printHeader();

    std::vector<ChainResult> pair_results;

    auto p2 = testChain<RevolutePairChainWithRotor<2, double>>("RevPairWithRotor", 2);
    pair_results.push_back(p2);
    printResult(p2);

    auto p4 = testChain<RevolutePairChainWithRotor<4, double>>("RevPairWithRotor", 4);
    pair_results.push_back(p4);
    printResult(p4);

    auto p6 = testChain<RevolutePairChainWithRotor<6, double>>("RevPairWithRotor", 6);
    pair_results.push_back(p6);
    printResult(p6);

    auto p8 = testChain<RevolutePairChainWithRotor<8, double>>("RevPairWithRotor", 8);
    pair_results.push_back(p8);
    printResult(p8);

    auto p10 = testChain<RevolutePairChainWithRotor<10, double>>("RevPairWithRotor", 10);
    pair_results.push_back(p10);
    printResult(p10);

    auto p12 = testChain<RevolutePairChainWithRotor<12, double>>("RevPairWithRotor", 12);
    pair_results.push_back(p12);
    printResult(p12);

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(pair_results, "RevolutePairChainWithRotor");

    // =========================================================================
    // Test 3: RevoluteTripleChainWithRotor (3 DOF per cluster)
    // =========================================================================
    std::cout << "Test 3: RevoluteTripleChainWithRotor - Triple Coupled (3 DOF per cluster)\n";
    printHeader();

    std::vector<ChainResult> triple_results;

    auto t3 = testChain<RevoluteTripleChainWithRotor<3, double>>("RevTripleWithRotor", 3);
    triple_results.push_back(t3);
    printResult(t3);

    auto t6 = testChain<RevoluteTripleChainWithRotor<6, double>>("RevTripleWithRotor", 6);
    triple_results.push_back(t6);
    printResult(t6);

    auto t9 = testChain<RevoluteTripleChainWithRotor<9, double>>("RevTripleWithRotor", 9);
    triple_results.push_back(t9);
    printResult(t9);

    auto t12 = testChain<RevoluteTripleChainWithRotor<12, double>>("RevTripleWithRotor", 12);
    triple_results.push_back(t12);
    printResult(t12);

    std::cout << std::string(86, '-') << "\n\n";
    analyzeScaling(triple_results, "RevoluteTripleChainWithRotor");

    // =========================================================================
    // Comparison at Same DOF
    // =========================================================================
    std::cout << "===========================================================================\n";
    std::cout << "Comparison: Performance at Same DOF Count\n";
    std::cout << "===========================================================================\n\n";

    // 6 DOF comparison
    std::cout << "6 DOF Systems:\n";
    std::cout << "  RevWithRotor (6 links, 6 clusters):      " << std::fixed << std::setprecision(2)
              << s6.avg_time_us << " us";
    if (s6.dof > 0 && p6.dof > 0) {
        std::cout << " (baseline)\n";
    } else {
        std::cout << "\n";
    }

    std::cout << "  RevPairWithRotor (6 links, 3 clusters):  " << std::fixed << std::setprecision(2)
              << p6.avg_time_us << " us";
    if (s6.dof > 0 && p6.dof > 0 && s6.avg_time_us > 0) {
        double ratio = p6.avg_time_us / s6.avg_time_us;
        std::cout << " (" << std::fixed << std::setprecision(1) << ratio << "x baseline)\n";
    } else {
        std::cout << "\n";
    }

    std::cout << "  RevTripleWithRotor (6 links, 2 clusters): " << std::fixed << std::setprecision(2)
              << t6.avg_time_us << " us";
    if (s6.dof > 0 && t6.dof > 0 && s6.avg_time_us > 0) {
        double ratio = t6.avg_time_us / s6.avg_time_us;
        std::cout << " (" << std::fixed << std::setprecision(1) << ratio << "x baseline)\n";
    } else {
        std::cout << "\n";
    }

    // 12 DOF comparison
    std::cout << "\n12 DOF Systems:\n";
    std::cout << "  RevWithRotor (12 links, 12 clusters):     " << std::fixed << std::setprecision(2)
              << s12.avg_time_us << " us";
    if (s12.dof > 0) {
        std::cout << " (baseline)\n";
    } else {
        std::cout << "\n";
    }

    std::cout << "  RevPairWithRotor (12 links, 6 clusters):  " << std::fixed << std::setprecision(2)
              << p12.avg_time_us << " us";
    if (s12.dof > 0 && p12.dof > 0 && s12.avg_time_us > 0) {
        double ratio = p12.avg_time_us / s12.avg_time_us;
        std::cout << " (" << std::fixed << std::setprecision(1) << ratio << "x baseline)\n";
    } else {
        std::cout << "\n";
    }

    std::cout << "  RevTripleWithRotor (12 links, 4 clusters): " << std::fixed << std::setprecision(2)
              << t12.avg_time_us << " us";
    if (s12.dof > 0 && t12.dof > 0 && s12.avg_time_us > 0) {
        double ratio = t12.avg_time_us / s12.avg_time_us;
        std::cout << " (" << std::fixed << std::setprecision(1) << ratio << "x baseline)\n";
    } else {
        std::cout << "\n";
    }

    // =========================================================================
    // Cluster Overhead Analysis
    // =========================================================================
    std::cout << "\n===========================================================================\n";
    std::cout << "Analysis: Cluster Complexity vs Number of Clusters\n";
    std::cout << "===========================================================================\n\n";

    std::cout << "Time per Cluster (6 DOF systems):\n";
    if (s6.num_clusters > 0)
        std::cout << "  RevWithRotor:      " << std::fixed << std::setprecision(2)
                  << s6.avg_time_us / s6.num_clusters << " us/cluster\n";
    if (p6.num_clusters > 0)
        std::cout << "  RevPairWithRotor:  " << std::fixed << std::setprecision(2)
                  << p6.avg_time_us / p6.num_clusters << " us/cluster\n";
    if (t6.num_clusters > 0)
        std::cout << "  RevTripleWithRotor: " << std::fixed << std::setprecision(2)
                  << t6.avg_time_us / t6.num_clusters << " us/cluster\n";

    std::cout << "\nObservations:\n";
    std::cout << "1. RevolutePair and RevoluteTriple mechanisms have higher per-cluster cost\n";
    std::cout << "   due to transmission modules and constraint Jacobian computations.\n";
    std::cout << "2. Fewer clusters (more DOF per cluster) may reduce algorithm overhead\n";
    std::cout << "   but increases per-cluster complexity.\n";
    std::cout << "3. The trade-off depends on the specific constraint structure and\n";
    std::cout << "   whether S_q caching is effective.\n\n";

    std::cout << "===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n";

    return 0;
}
