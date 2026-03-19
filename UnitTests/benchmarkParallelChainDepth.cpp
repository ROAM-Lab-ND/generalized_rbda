#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>
#include <numeric>
#include <random>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "config.h"

using namespace grbda;

// Fixed seed for reproducibility
constexpr unsigned int RANDOM_SEED = 42;

// ============================================================================
// Parallel Chain Branch Depth Benchmark
// ============================================================================
// This benchmark tests how computational cost changes based on:
// 1. Two parallel chains of identical length sharing a base joint
// 2. Branch structures added at varying depths along chain1
//
// The structure is:
//        Base (root)
//       /          \
//    Chain1        Chain2
//     link1         link1
//      |--branch1 (3 links)
//     link2         link2
//      |--branch2 (3 links)
//     ...
//
// The goal is to understand how tree depth and branching
// affects derivative computation cost.
// ============================================================================

struct DepthResult {
    std::string config;
    int chain_length;           // Length of each chain
    int num_cross_links;        // Number of cross-links connecting the chains
    int cross_link_depths;      // Depths at which cross-links are placed (e.g., 1, 1-2, 1-2-3)
    int dof;
    double avg_time_us;
    double std_time_us;         // Standard deviation for noise assessment
    double median_time_us;      // Median (more robust to outliers)
    double min_time_us;         // Minimum time (best estimate of true time)
    double max_error_dq;
    double max_error_dqdot;
};

// Compute statistics from a vector of timing samples
struct TimingStats {
    double mean;
    double median;
    double std_dev;
    double min;
    double max;
    double trimmed_mean;  // Mean after removing top/bottom 10%
};

TimingStats computeStats(std::vector<double>& samples) {
    TimingStats stats;
    size_t n = samples.size();
    if (n == 0) return {0, 0, 0, 0, 0, 0};

    // Sort for median and percentiles
    std::sort(samples.begin(), samples.end());

    stats.min = samples.front();
    stats.max = samples.back();
    stats.median = (n % 2 == 0)
        ? (samples[n/2 - 1] + samples[n/2]) / 2.0
        : samples[n/2];

    // Mean
    stats.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / n;

    // Standard deviation
    double sq_sum = 0.0;
    for (double s : samples) {
        sq_sum += (s - stats.mean) * (s - stats.mean);
    }
    stats.std_dev = std::sqrt(sq_sum / n);

    // Trimmed mean (remove top and bottom 10%)
    size_t trim_count = n / 10;
    if (n > 20 && trim_count > 0) {
        double trimmed_sum = 0.0;
        size_t trimmed_n = n - 2 * trim_count;
        for (size_t i = trim_count; i < n - trim_count; ++i) {
            trimmed_sum += samples[i];
        }
        stats.trimmed_mean = trimmed_sum / trimmed_n;
    } else {
        stats.trimmed_mean = stats.mean;
    }

    return stats;
}

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

// Build two parallel chains of length N sharing a base, with cross-links
// (RevolutePair constraints) at specified depths
// cross_link_depths: set of depths where cross-links should be placed
// Example: cross_link_depths = {1, 3} places cross-links at depth 1 and 3
ClusterTreeModel<double> buildParallelChainsWithCrossLinks(
    int chain_length,
    const std::set<int>& cross_link_depths) {
    
    ClusterTreeModel<double> model{};

    Mat3<double> I3 = Mat3<double>::Identity();
    Vec3<double> z3 = Vec3<double>::Zero();

    model.setGravity(Vec3<double>{9.81, 0., 0.});

    Mat3<double> link_inertia;
    link_inertia << 0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    const SpatialInertia<double> link_spatial_inertia(1.0, Vec3<double>(0.5, 0., 0.), link_inertia);

    ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

    // Build base link
    std::string base_name = "base";
    spatial::Transform<double> base_Xtree(I3, z3);
    model.template appendBody<ClusterJoints::Revolute<double>>(
        base_name, link_spatial_inertia, "ground", base_Xtree, axis);

    // Build two parallel chains
    std::vector<std::string> chain1_links;
    std::vector<std::string> chain2_links;
    
    std::string prev_chain1 = base_name;
    std::string prev_chain2 = base_name;

    for (int i = 1; i <= chain_length; ++i) {
        // Chain 1 link
        std::string link1_name = "chain1_link" + std::to_string(i);
        spatial::Transform<double> link1_Xtree(I3, Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link1_name, link_spatial_inertia, prev_chain1, link1_Xtree, axis);
        chain1_links.push_back(link1_name);
        prev_chain1 = link1_name;

        // Chain 2 link
        std::string link2_name = "chain2_link" + std::to_string(i);
        spatial::Transform<double> link2_Xtree(I3, Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link2_name, link_spatial_inertia, prev_chain2, link2_Xtree, axis);
        chain2_links.push_back(link2_name);
        prev_chain2 = link2_name;

        // Add cross-link at this depth if specified  
        if (cross_link_depths.count(i)) {
            // Create a loop constraint at this depth
            // Use RevolutePair at the same depth on both chains to create the loop
            std::string pair_cluster_name = "loop_revpair_depth" + std::to_string(i);
            
            // Create two link pairs - one from each chain
            std::string link_a1 = "loop_linkA1_depth" + std::to_string(i);
            std::string link_a2 = "loop_linkA2_depth" + std::to_string(i);
            std::string link_b1 = "loop_linkB1_depth" + std::to_string(i);
            std::string link_b2 = "loop_linkB2_depth" + std::to_string(i);
            
            // Both links descend from chain1_link_i to form constraint pair
            spatial::Transform<double> link_a1_Xtree(I3, Vec3<double>(0.0, 1.0, 0.));
            spatial::Transform<double> link_a2_Xtree(I3, Vec3<double>(0.0, -1.0, 0.));
            
            auto body_a1 = model.registerBody(link_a1, link_spatial_inertia,
                                            link1_name, link_a1_Xtree);
            auto body_a2 = model.registerBody(link_a2, link_spatial_inertia,
                                            link1_name, link_a2_Xtree);
            
            // Create constraint joints
            typedef grbda::ClusterJoints::RevolutePairWithRotor<double> RevPairRotor;
            typedef grbda::ClusterJoints::ParallelBeltTransmissionModule<1, double> ProxTransModule;
            typedef grbda::ClusterJoints::ParallelBeltTransmissionModule<2, double> DistTransModule;
            
            // Create rotor bodies for the RevolutePair
            std::string rotor_a = "loop_rotorA_depth" + std::to_string(i);
            std::string rotor_b = "loop_rotorB_depth" + std::to_string(i);
            
            Mat3<double> rotor_inertia;
            rotor_inertia << 0., 0., 0., 0., 0., 0., 0., 0., 1e-4;
            SpatialInertia<double> rotor_spatial_inertia(0., Vec3<double>::Zero(), rotor_inertia);
            
            auto rotor_body_a = model.registerBody(rotor_a, rotor_spatial_inertia,
                                                  link1_name, link_a1_Xtree);
            auto rotor_body_b = model.registerBody(rotor_b, rotor_spatial_inertia,
                                                  link1_name, link_a2_Xtree);
            
            // Create RevolutePair modules
            ProxTransModule moduleA{body_a1, rotor_body_a, axis, axis, 2.0, Vec1<double>{3.0}};
            DistTransModule moduleB{body_a2, rotor_body_b, axis, axis, 2.0, Vec2<double>{3.0, 1.0}};
            
            // Append as cluster - this creates the implicit loop constraint
            model.template appendRegisteredBodiesAsCluster<RevPairRotor>(pair_cluster_name, moduleA, moduleB);
        }
    }

    return model;
}

// Build two parallel chains without cross-links (baseline)
ClusterTreeModel<double> buildParallelChainsBaseline(int chain_length) {
    ClusterTreeModel<double> model{};

    Mat3<double> I3 = Mat3<double>::Identity();
    Vec3<double> z3 = Vec3<double>::Zero();

    model.setGravity(Vec3<double>{9.81, 0., 0.});

    Mat3<double> link_inertia;
    link_inertia << 0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1;
    const SpatialInertia<double> link_spatial_inertia(1.0, Vec3<double>(0.5, 0., 0.), link_inertia);

    ori::CoordinateAxis axis = ori::CoordinateAxis::Z;

    // Build base link
    std::string base_name = "base";
    spatial::Transform<double> base_Xtree(I3, z3);
    model.template appendBody<ClusterJoints::Revolute<double>>(
        base_name, link_spatial_inertia, "ground", base_Xtree, axis);

    // Build two parallel chains
    std::string prev_chain1 = base_name;
    std::string prev_chain2 = base_name;

    for (int i = 1; i <= chain_length; ++i) {
        // Chain 1 link
        std::string link1_name = "chain1_link" + std::to_string(i);
        spatial::Transform<double> link1_Xtree(I3, Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link1_name, link_spatial_inertia, prev_chain1, link1_Xtree, axis);
        prev_chain1 = link1_name;

        // Chain 2 link
        std::string link2_name = "chain2_link" + std::to_string(i);
        spatial::Transform<double> link2_Xtree(I3, Vec3<double>(1.0, 0., 0.));
        model.template appendBody<ClusterJoints::Revolute<double>>(
            link2_name, link_spatial_inertia, prev_chain2, link2_Xtree, axis);
        prev_chain2 = link2_name;
    }

    return model;
}

DepthResult testModel(ClusterTreeModel<double>& model, const std::string& config,
                      int chain_length, int num_cross_links, int cross_link_depths,
                      bool print_debug = false) {
    try {
        int nDOF = model.getNumDegreesOfFreedom();
        int nClusters = model.clusters().size();

        if (print_debug) {
            // Find position of the RevolutePair cluster (if any)
            int revpair_pos = -1;
            int idx = 0;
            for (const auto& cluster : model.clusters()) {
                if (cluster->name_.find("loop_revpair") != std::string::npos) {
                    revpair_pos = idx;
                    break;
                }
                idx++;
            }
            std::cout << "  " << config << ": DOF=" << nDOF << ", clusters=" << nClusters
                      << ", revpair_cluster_pos=" << revpair_pos << "\n";
        }

        if (nDOF == 0) {
            throw std::runtime_error("Model has zero DOF");
        }

        // Use fixed seed for reproducible random state across all models
        std::mt19937 rng(RANDOM_SEED);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        // Set deterministic state
        ModelState<double> model_state;
        for (const auto& cluster : model.clusters()) {
            JointState<double> js;
            js.position = DVec<double>(cluster->num_positions_);
            js.velocity = DVec<double>(cluster->num_velocities_);
            for (int i = 0; i < cluster->num_positions_; ++i) {
                js.position(i) = dist(rng) * 0.5;  // Small angles to avoid singularities
            }
            for (int i = 0; i < cluster->num_velocities_; ++i) {
                js.velocity(i) = dist(rng);
            }
            model_state.push_back(js);
        }
        model.setState(model_state);
        auto [q, qd] = model.getState();

        // Deterministic acceleration
        DVec<double> ydd(nDOF);
        for (int i = 0; i < nDOF; ++i) {
            ydd(i) = dist(rng);
        }

        // Extended warmup phase - ensure CPU is in steady state
        const int warmup_iterations = 2000;
        for (int i = 0; i < warmup_iterations; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }

        // Aggressive noise reduction strategy:
        // Run multiple complete measurement sweeps and aggregate
        const int num_sweeps = 5;         // Number of complete measurement sweeps
        const int samples_per_sweep = 200; // Samples per sweep
        const int batch_size = 100;        // Iterations per sample (reduces timer overhead)

        std::vector<double> all_samples;
        all_samples.reserve(num_sweeps * samples_per_sweep);

        for (int sweep = 0; sweep < num_sweeps; ++sweep) {
            // Small pause between sweeps to let system settle
            // (busy wait to avoid sleep syscall overhead affecting subsequent timing)
            volatile int dummy = 0;
            for (int i = 0; i < 100000; ++i) { dummy += i; }
            (void)dummy;

            // Re-warmup between sweeps
            for (int i = 0; i < 100; ++i) {
                auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
                (void)dtau_dq;
                (void)dtau_dqdot;
            }

            for (int s = 0; s < samples_per_sweep; ++s) {
                auto start = std::chrono::high_resolution_clock::now();
                for (int i = 0; i < batch_size; ++i) {
                    auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
                    (void)dtau_dq;
                    (void)dtau_dqdot;
                }
                auto end = std::chrono::high_resolution_clock::now();

                double batch_time_us = std::chrono::duration<double, std::micro>(end - start).count();
                all_samples.push_back(batch_time_us / batch_size);
            }
        }

        // Compute statistics from all sweeps combined
        TimingStats stats = computeStats(all_samples);

        // Accuracy check
        auto [dtau_dq_analytical, dtau_dqdot_analytical] =
            model.firstOrderInverseDynamicsDerivatives(ydd);
        auto [dtau_dq_numerical, dtau_dqdot_numerical] =
            computeFiniteDifferenceDerivatives(model, q, qd, ydd);

        DMat<double> error_dq = (dtau_dq_analytical - dtau_dq_numerical).cwiseAbs();
        DMat<double> error_dqdot = (dtau_dqdot_analytical - dtau_dqdot_numerical).cwiseAbs();

        // Return all statistics - min is often the best estimate of true time
        return {config, chain_length, num_cross_links, cross_link_depths, nDOF,
                stats.trimmed_mean, stats.std_dev, stats.median, stats.min,
                error_dq.maxCoeff(), error_dqdot.maxCoeff()};

    } catch (const std::exception& e) {
        std::cerr << "Error testing " << config << ": " << e.what() << "\n";
        return {config, chain_length, num_cross_links, cross_link_depths, -1, 0, 0, 0, 0, 0, 0};
    }
}

void printHeader() {
    std::cout << std::left << std::setw(20) << "Configuration"
              << std::setw(6) << "CLen"
              << std::setw(6) << "XLink"
              << std::setw(6) << "Depth"
              << std::setw(5) << "DOF"
              << std::setw(12) << "Min (us)"
              << std::setw(12) << "Mean"
              << std::setw(12) << "Median"
              << std::setw(12) << "Err dq"
              << "\n";
    std::cout << std::string(105, '-') << "\n";
}

void printResult(const DepthResult& r) {
    if (r.dof > 0) {
        std::cout << std::left << std::setw(20) << r.config
                  << std::setw(6) << r.chain_length
                  << std::setw(6) << r.num_cross_links
                  << std::setw(6) << r.cross_link_depths
                  << std::setw(5) << r.dof
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.min_time_us
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.avg_time_us
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.median_time_us
                  << std::setw(12) << std::scientific << std::setprecision(2) << r.max_error_dq
                  << "\n";
    } else {
        std::cout << std::left << std::setw(20) << r.config
                  << std::setw(6) << r.chain_length
                  << std::setw(6) << r.num_cross_links
                  << std::setw(6) << r.cross_link_depths
                  << std::setw(5) << "N/A"
                  << std::setw(12) << "FAILED"
                  << std::setw(12) << ""
                  << std::setw(12) << ""
                  << std::setw(12) << ""
                  << "\n";
    }
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Parallel Chain Cross-Link Depth Benchmark\n";
    std::cout << "===========================================================================\n\n";

    std::cout << "Testing two parallel chains with cross-links (RevolutePair constraints)\n";
    std::cout << "at increasing depths to measure impact on derivative computation.\n\n";

    // =========================================================================
    // Randomized multi-pass benchmarking strategy
    // =========================================================================
    // To reduce temporal noise (thermal throttling, system activity), we:
    // 1. Create all test configurations upfront
    // 2. Run multiple passes through all configs in randomized order
    // 3. Aggregate results using minimum time (best estimate of true time)

    const int NUM_PASSES = 5;  // Number of complete passes

    // Define all configurations: (depth, is_baseline)
    // depth=0 means baseline (no cross-link)
    std::vector<int> all_depths;
    all_depths.push_back(0);  // Baseline
    for (int d = 1; d <= 40; ++d) {
        all_depths.push_back(d);
    }

    // Storage for results across passes: depth -> vector of min_times from each pass
    std::map<int, std::vector<double>> pass_min_times;
    std::map<int, DepthResult> best_results;  // Store best result per depth

    std::cout << "Running " << NUM_PASSES << " randomized passes through all "
              << all_depths.size() << " configurations...\n\n";

    std::mt19937 shuffle_rng(RANDOM_SEED + 1000);  // Different seed for shuffling

    for (int pass = 0; pass < NUM_PASSES; ++pass) {
        bool debug_pass = (pass == 0);  // Print debug info on first pass only
        if (debug_pass) {
            std::cout << "Pass " << (pass + 1) << "/" << NUM_PASSES << " (with diagnostics)...\n";
        } else {
            std::cout << "Pass " << (pass + 1) << "/" << NUM_PASSES << "... " << std::flush;
        }

        // NO SHUFFLE - run in sequential order to diagnose timing issues
        std::vector<int> shuffled_depths = all_depths;
        // std::shuffle(shuffled_depths.begin(), shuffled_depths.end(), shuffle_rng);

        for (int depth : shuffled_depths) {
            DepthResult result;
            if (depth == 0) {
                auto model = buildParallelChainsBaseline(40);
                result = testModel(model, "Baseline_40L", 40, 0, 0, debug_pass);
            } else {
                auto model = buildParallelChainsWithCrossLinks(40, {depth});
                std::string label = "Depth" + std::to_string(depth) + "_40L";
                result = testModel(model, label, 40, 1, depth, debug_pass);
            }

            pass_min_times[depth].push_back(result.min_time_us);

            // Print per-pass timing for key depths to diagnose variability
            if (depth == 0 || depth == 1 || depth == 4 || depth == 18 ||
                depth == 26 || depth == 27 || depth == 40) {
                std::cout << "    Pass " << (pass+1) << " Depth " << depth
                          << ": min=" << std::fixed << std::setprecision(2) << result.min_time_us << " us\n";
            }

            // Keep track of best (lowest min) result for each depth
            if (best_results.find(depth) == best_results.end() ||
                result.min_time_us < best_results[depth].min_time_us) {
                best_results[depth] = result;
            }
        }
        if (!debug_pass) {
            std::cout << "done\n";
        }
    }

    std::cout << "\n";

    // =========================================================================
    // Aggregate results: use MEDIAN of minimums across all passes
    // (Median is more robust to outliers than minimum or mean)
    // =========================================================================
    std::vector<DepthResult> all_results;

    std::cout << "40-Link Parallel Chains - Single Cross-Link Depth Sweep\n";
    std::cout << "(Median of " << NUM_PASSES << " passes)\n";
    printHeader();

    // Process in order (baseline first, then depths 1-40)
    for (int depth : all_depths) {
        DepthResult& r = best_results[depth];

        // Compute statistics across passes for this depth
        std::vector<double> times = pass_min_times[depth];  // Copy for sorting
        std::sort(times.begin(), times.end());

        size_t n = times.size();
        double median_of_mins = (n % 2 == 0)
            ? (times[n/2 - 1] + times[n/2]) / 2.0
            : times[n/2];

        double sum = 0;
        for (double t : times) sum += t;
        double mean_of_mins = sum / n;

        // Update the result with the MEDIAN of minimums (more robust)
        r.min_time_us = median_of_mins;
        r.avg_time_us = mean_of_mins;  // Mean of min times across passes

        all_results.push_back(r);
        printResult(r);
    }

    std::cout << std::string(106, '-') << "\n\n";

    // =========================================================================
    // Analysis
    // =========================================================================
    std::cout << "===========================================================================\n";
    std::cout << "Analysis Summary\n";
    std::cout << "===========================================================================\n\n";

    // Group by chain length
    std::map<int, std::vector<DepthResult>> by_chain_length;
    for (const auto& r : all_results) {
        if (r.dof > 0) {
            by_chain_length[r.chain_length].push_back(r);
        }
    }

    for (const auto& [chain_len, results] : by_chain_length) {
        std::cout << "Chain Length " << chain_len << ":\n";

        if (!results.empty()) {
            // Use median as baseline (more robust to outliers)
            double baseline = results[0].median_time_us;
            for (const auto& r : results) {
                double ratio = r.median_time_us / baseline;
                double cv = (r.avg_time_us > 0) ? (r.std_time_us / r.avg_time_us * 100) : 0;
                std::cout << "  " << r.config << ": "
                          << std::fixed << std::setprecision(2) << r.median_time_us << " us";
                if (r.num_cross_links > 0) {
                    std::cout << " (" << std::fixed << std::setprecision(2) << ratio << "x)";
                } else {
                    std::cout << " (baseline)";
                }
                std::cout << " [CV: " << std::fixed << std::setprecision(1) << cv << "%]\n";
            }
        }
        std::cout << "\n";
    }

    std::cout << "Key Observations:\n";
    std::cout << "1. Cost increase from adding a single cross-link at different depths\n";
    std::cout << "2. Whether depth position affects the computational cost (early vs late)\n";
    std::cout << "3. Continuous depth sweep from 1 to 40 to identify patterns\n";
    std::cout << "4. Whether cost scales linearly or nonlinearly with cross-link depth\n";
    std::cout << "5. Error should remain bounded (~1e-7) regardless of configuration\n\n";

    std::cout << "===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n";

    // =========================================================================
    // Export results to CSV files
    // =========================================================================
    std::string output_dir = std::string(SOURCE_DIRECTORY) + "/../benchmark_figures/data/";

    // Export all results to a single CSV
    {
        std::ofstream csv(output_dir + "parallel_chain_depth.csv");
        csv << "config,chain_length,num_cross_links,cross_link_depth,dof,mean_us,median_us,std_us,max_err_dq,max_err_dqd\n";
        for (const auto& r : all_results) {
            if (r.dof > 0) {
                csv << r.config << "," << r.chain_length << "," << r.num_cross_links << ","
                    << r.cross_link_depths << "," << r.dof << ","
                    << std::fixed << std::setprecision(4) << r.avg_time_us << ","
                    << std::fixed << std::setprecision(4) << r.median_time_us << ","
                    << std::fixed << std::setprecision(4) << r.std_time_us << ","
                    << std::scientific << std::setprecision(2) << r.max_error_dq << ","
                    << r.max_error_dqdot << "\n";
            }
        }
        std::cout << "Exported: " << output_dir << "parallel_chain_depth.csv\n";
    }

    // Export 40-link single cross-link depth sweep for Figure 7 left panel
    {
        std::ofstream csv(output_dir + "loop_depth_40L.csv");
        csv << "chain_length,cross_link_depth,num_cross_links,dof,mean_us,median_us,std_us,min_us\n";
        for (const auto& r : all_results) {
            if (r.dof > 0 && r.chain_length == 40 && r.num_cross_links <= 1) {
                csv << r.chain_length << "," << r.cross_link_depths << "," << r.num_cross_links << ","
                    << r.dof << ","
                    << std::fixed << std::setprecision(4) << r.avg_time_us << ","
                    << std::fixed << std::setprecision(4) << r.median_time_us << ","
                    << std::fixed << std::setprecision(4) << r.std_time_us << ","
                    << std::fixed << std::setprecision(4) << r.min_time_us << "\n";
            }
        }
        std::cout << "Exported: " << output_dir << "loop_depth_40L.csv\n";
    }

    // Export multi-chain comparison for Figure 7 right panel
    {
        std::ofstream csv(output_dir + "loop_depth_multi_chain.csv");
        csv << "chain_length,num_cross_links,config,dof,mean_us,median_us,std_us\n";
        for (const auto& r : all_results) {
            if (r.dof > 0) {
                csv << r.chain_length << "," << r.num_cross_links << "," << r.config << ","
                    << r.dof << ","
                    << std::fixed << std::setprecision(4) << r.avg_time_us << ","
                    << std::fixed << std::setprecision(4) << r.median_time_us << ","
                    << std::fixed << std::setprecision(4) << r.std_time_us << "\n";
            }
        }
        std::cout << "Exported: " << output_dir << "loop_depth_multi_chain.csv\n";
    }

    return 0;
}
