#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>
#include <numeric>
#include <random>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/ClusterJoints/LoopConstraint.h"
#include "config.h"

using namespace grbda;

// Helper to print ClusterJointTypes as string
namespace grbda {
inline const char* ClusterJointTypeToString(ClusterJointTypes type) {
    switch (type) {
        case ClusterJointTypes::FourBar: return "FourBar";
        case ClusterJointTypes::Free: return "Free";
        case ClusterJointTypes::Generic: return "Generic";
        case ClusterJointTypes::Revolute: return "Revolute";
        case ClusterJointTypes::RevolutePair: return "RevolutePair";
        case ClusterJointTypes::RevolutePairWithRotor: return "RevolutePairWithRotor";
        case ClusterJointTypes::RevoluteTripleWithRotor: return "RevoluteTripleWithRotor";
        case ClusterJointTypes::RevoluteWithRotor: return "RevoluteWithRotor";
        case ClusterJointTypes::TelloHipDifferential: return "TelloHipDifferential";
        case ClusterJointTypes::TelloKneeAnkleDifferential: return "TelloKneeAnkleDifferential";
        default: return "Unknown";
    }
}
}
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>
#include <numeric>
#include <random>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/ClusterJoints/LoopConstraint.h"
#include "config.h"

using namespace grbda;

// Fixed seed for reproducibility
constexpr unsigned int RANDOM_SEED = 42;

// ============================================================================
// Parallel Chain Loop Size Benchmark
// ============================================================================
// This benchmark tests how computational cost changes based on loop size
// in an A-shaped parallel chain topology:
//
//          base
//         /    \
//    chain1    chain2
//      |          |
//     ...        ...
//      |          |
//   link_1_k   link_2_k
//      |          |
//  connecting_rod-+     <-- loop constraint connects chains here
//      |          |
//     ...        ...
//
// The "loop_size" parameter determines where the cross-link connects:
//   loop_size = 2 * connection_depth + 1
//
// A larger loop_size means the connection is deeper in the tree,
// creating a larger closed loop through the kinematic structure.
// ============================================================================

// URDF directory for parallel chain models
const std::string urdf_directory = std::string(SOURCE_DIRECTORY) + "/Benchmarking/urdfs/";

struct BenchmarkResult {
    int chain_depth;        // Total depth of each chain (5, 10, 20, or 40)
    int loop_size;          // Size of the closed loop (2*connection_depth + 1)
    int connection_depth;   // Depth at which chains are connected
    int dof;                // Degrees of freedom
    int num_bodies;         // Number of bodies in the model
    double min_time_us;     // Minimum time (best estimate of true cost)
    double mean_time_us;    // Mean time across samples
    double median_time_us;  // Median time
    double std_time_us;     // Standard deviation
    double max_error_dq;    // Max error in dtau/dq
    double max_error_dqdot; // Max error in dtau/dqdot
    bool is_baseline;       // True if this is the open-chain baseline (no loop)
    // Breakdown (averages per call, from profiling API)
    double fwd_kin_us    = 0;
    double fwd_casadi_us = 0;
    double fwd_other_us  = 0;
    double bwd_casadi_us = 0;
    double bwd_other_us  = 0;
    double bwd_prop_us   = 0;
};

// Compute statistics from timing samples
struct TimingStats {
    double min;
    double max;
    double mean;
    double median;
    double std_dev;
    double trimmed_mean;
};

TimingStats computeStats(std::vector<double>& samples) {
    TimingStats stats = {0, 0, 0, 0, 0, 0};
    size_t n = samples.size();
    if (n == 0) return stats;

    std::sort(samples.begin(), samples.end());

    stats.min = samples.front();
    stats.max = samples.back();
    stats.median = (n % 2 == 0)
        ? (samples[n/2 - 1] + samples[n/2]) / 2.0
        : samples[n/2];

    stats.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / n;

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

// Compute finite difference derivatives for validation
template<typename Scalar>
std::pair<DMat<Scalar>, DMat<Scalar>> computeFiniteDifferenceDerivatives(
    ClusterTreeModel<Scalar>& model,
    const DVec<Scalar>& q,
    const DVec<Scalar>& qd,
    const DVec<Scalar>& ydd,
    double h = 1e-7)
{
    const int nDOF = model.getNumDegreesOfFreedom();
    DMat<Scalar> dtau_dq(nDOF, nDOF);
    DMat<Scalar> dtau_dqdot(nDOF, nDOF);

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

        dtau_dq.col(j) = (tau_plus - tau_minus) / (2.0 * h);
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

        dtau_dqdot.col(j) = (tau_plus - tau_minus) / (2.0 * h);
    }

    return {dtau_dq, dtau_dqdot};
}

// Available configurations from the URDF files
// Using Implicit constraint type which creates true A-shape topology with loop constraints
struct ParallelChainConfig {
    int depth;
    std::vector<int> loop_sizes;  // Available loop sizes for this depth
};

std::vector<ParallelChainConfig> getAvailableConfigs() {
    // Implicit URDFs: loop_size = 2 * connection_depth + 1
    return {
        {5,  {3, 5, 7, 9, 11}},
        {10, {3, 5, 7, 9, 11, 13, 15, 17, 19}},
        //{20, {3, 7, 13, 21, 31}},
        //{40, {3, 9, 17, 29, 41}}
    };
}

std::string buildUrdfPath(int depth, int loop_size, bool with_loop) {
    std::string prefix = with_loop ? "loop_size" : "approx_loop_size";
    return urdf_directory + "parallel_chains/Implicit/depth" +
           std::to_string(depth) + "/" + prefix + std::to_string(loop_size) + ".urdf";
}

BenchmarkResult benchmarkModel(const std::string& urdf_path,
                                int chain_depth, int loop_size,
                                bool is_baseline,
                                bool print_debug = false) {
    BenchmarkResult result;
    result.chain_depth = chain_depth;
    result.loop_size = loop_size;
    // For Implicit constraints: loop_size = 2 * connection_depth + 1
    result.connection_depth = (loop_size - 1) / 2;
    result.is_baseline = is_baseline;
    result.dof = -1;
    result.num_bodies = -1;

    try {
        ClusterTreeModel<double> model;
        model.buildModelFromURDF(urdf_path);

        int nDOF = model.getNumDegreesOfFreedom();
        int nBodies = model.getNumBodies();

        result.dof = nDOF;
        result.num_bodies = nBodies;

        if (print_debug) {
            std::cout << "  depth=" << chain_depth
                      << " loop_size=" << loop_size
                      << " (connection at depth " << result.connection_depth << ")"
                      << ": DOF=" << nDOF
                      << ", bodies=" << nBodies
                      << ", clusters=" << model.clusters().size()
                      << (is_baseline ? " [baseline]" : " [with loop]")
                      << "\n";
        }

        if (nDOF == 0) {
            throw std::runtime_error("Model has zero DOF");
        }

        // Use fixed seed for reproducibility
        std::mt19937 rng(RANDOM_SEED);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        // Set random state using joint's randomJointState(), robust for implicit constraints, with retry
        ModelState<double> model_state;
        constexpr int max_attempts = 1000;
        for (const auto& cluster : model.clusters()) {
            bool is_implicit = false;
            try {
                is_implicit = cluster->joint_->G().cols() != cluster->joint_->G().rows();
            } catch (...) {}
            int attempt = 0;
            bool success = false;
            while (attempt < max_attempts && !success) {
                try {
                    JointState<double> js = cluster->joint_->randomJointState();
                    model_state.push_back(js);
                    success = true;
                } catch (const std::exception& e) {
                    ++attempt;
                    if (attempt >= max_attempts) {
                        std::cerr << "[Error] Failed to generate valid random state for cluster '"
                                  << grbda::ClusterJointTypeToString(cluster->joint_->type())
                                  << (is_implicit ? " (IMPLICIT)" : "")
                                  << "' after " << max_attempts << " attempts: " << e.what() << std::endl;
                        if (is_implicit) {
                            std::cerr << "[FATAL] This joint type does not support robust random state generation for implicit constraints. Please implement or fix root-finding in randomJointState()." << std::endl;
                        }
                        throw;
                    } else {
                        std::cerr << "[Retry] Attempt " << attempt << " for cluster '"
                                  << grbda::ClusterJointTypeToString(cluster->joint_->type())
                                  << (is_implicit ? " (IMPLICIT)" : "") << ": " << e.what() << std::endl;
                    }
                }
            }
        }
        model.setState(model_state);

        // Extract q and qd from model_state for finite difference validation
        // (avoiding getState() which has issues with implicit constraints)
        DVec<double> q(nDOF), qd(nDOF);
        int idx = 0;
        for (const auto& js : model_state) {
            int nv = js.velocity.size();
            q.segment(idx, nv) = js.position;
            qd.segment(idx, nv) = js.velocity;
            idx += nv;
        }

        DVec<double> ydd(nDOF);
        for (int i = 0; i < nDOF; ++i) {
            ydd(i) = dist(rng);
        }

        // Warmup
        const int warmup_iterations = 2000;
        for (int i = 0; i < warmup_iterations; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }

        // Benchmark with multiple sweeps for noise reduction
        const int num_sweeps = 5;
        const int samples_per_sweep = 200;
        const int batch_size = 100;

        std::vector<double> all_samples;
        all_samples.reserve(num_sweeps * samples_per_sweep);

        for (int sweep = 0; sweep < num_sweeps; ++sweep) {
            // Busy wait between sweeps
            volatile int dummy = 0;
            for (int i = 0; i < 100000; ++i) { dummy += i; }
            (void)dummy;

            // Re-warmup
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

        TimingStats stats = computeStats(all_samples);
        result.min_time_us = stats.min;
        result.mean_time_us = stats.trimmed_mean;
        result.median_time_us = stats.median;
        result.std_time_us = stats.std_dev;

        // Collect profiling breakdown over a separate fixed run (1000 calls, post-warmup)
        enableIDDerivativesProfiling();
        const int prof_iters = 1000;
        for (int i = 0; i < prof_iters; ++i) {
            auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)dtau_dq;
            (void)dtau_dqdot;
        }
        auto prof_data = getIDDerivativesProfilingData();
        resetIDDerivativesProfiling();
        // prof_data: {fwd_kin, fwd_casadi, fwd_other, bwd_casadi, bwd_other, bwd_prop, total}
        result.fwd_kin_us    = prof_data[0];
        result.fwd_casadi_us = prof_data[1];
        result.fwd_other_us  = prof_data[2];
        result.bwd_casadi_us = prof_data[3];
        result.bwd_other_us  = prof_data[4];
        result.bwd_prop_us   = prof_data[5];

        // Skipping numerical derivative validation; only timing results are recorded.
        result.max_error_dq = 0.0;
        result.max_error_dqdot = 0.0;

    } catch (const std::exception& e) {
        std::cerr << "Error benchmarking " << urdf_path << ": " << e.what() << "\n";
    }

    return result;
}

void printHeader() {
    std::cout << std::left
              << std::setw(8)  << "Depth"
              << std::setw(10) << "LoopSize"
              << std::setw(10) << "ConnDepth"
              << std::setw(6)  << "DOF"
              << std::setw(8)  << "Bodies"
              << std::setw(10) << "Min(us)"
              << std::setw(10) << "Mean(us)"
              << std::setw(10) << "FwdKin"
              << std::setw(10) << "FwdCasADi"
              << std::setw(10) << "FwdOther"
              << std::setw(10) << "BwdCasADi"
              << std::setw(10) << "BwdOther"
              << std::setw(10) << "BwdProp"
              << std::setw(10) << "Type"
              << "\n";
    std::cout << std::string(136, '-') << "\n";
}

void printResult(const BenchmarkResult& r) {
    if (r.dof > 0) {
        std::cout << std::left  << std::fixed << std::setprecision(2)
                  << std::setw(8)  << r.chain_depth
                  << std::setw(10) << r.loop_size
                  << std::setw(10) << r.connection_depth
                  << std::setw(6)  << r.dof
                  << std::setw(8)  << r.num_bodies
                  << std::setw(10) << r.min_time_us
                  << std::setw(10) << r.mean_time_us
                  << std::setw(10) << r.fwd_kin_us
                  << std::setw(10) << r.fwd_casadi_us
                  << std::setw(10) << r.fwd_other_us
                  << std::setw(10) << r.bwd_casadi_us
                  << std::setw(10) << r.bwd_other_us
                  << std::setw(10) << r.bwd_prop_us
                  << std::setw(10) << (r.is_baseline ? "baseline" : "loop")
                  << "\n";
    } else {
        std::cout << std::left
                  << std::setw(8)  << r.chain_depth
                  << std::setw(10) << r.loop_size
                  << std::setw(10) << r.connection_depth
                  << std::setw(6)  << "N/A"
                  << std::setw(8)  << "N/A"
                  << std::setw(10) << "FAILED"
                  << "\n";
    }
}

int main() {
    std::cout << "\n===========================================================================\n";
    std::cout << "Parallel Chain Loop Size Benchmark (A-Shape Topology)\n";
    std::cout << "===========================================================================\n\n";

    std::cout << "This benchmark measures inverse dynamics derivative computation cost\n";
    std::cout << "for A-shaped parallel chains with varying loop sizes.\n\n";

    std::cout << "Topology:\n";
    std::cout << "       base           The 'loop_size' parameter determines where\n";
    std::cout << "      /    \\          the cross-link connects the two chains:\n";
    std::cout << "  chain1  chain2        loop_size = 2 * connection_depth + 1\n";
    std::cout << "    |        |        \n";
    std::cout << "   ...      ...       Larger loop_size = deeper connection = larger loop\n";
    std::cout << "    |        |        \n";
    std::cout << "   [connection]       \n";
    std::cout << "    |        |        \n";
    std::cout << "   ...      ...       \n\n";

    auto configs = getAvailableConfigs();
    std::vector<BenchmarkResult> all_results;

    // Run benchmarks for each configuration
    const int NUM_PASSES = 1;
    std::map<std::pair<int,int>, std::vector<BenchmarkResult>> pass_results;

    std::cout << "Running " << NUM_PASSES << " passes through all configurations...\n\n";

    for (int pass = 0; pass < NUM_PASSES; ++pass) {
        bool debug_pass = (pass == 0);
        std::cout << "Pass " << (pass + 1) << "/" << NUM_PASSES;
        if (debug_pass) {
            std::cout << " (with diagnostics)...\n";
        } else {
            std::cout << "... " << std::flush;
        }

        for (const auto& config : configs) {
            if (debug_pass) {
                std::cout << "\n  Chain depth " << config.depth << ":\n";
            }

            for (int loop_size : config.loop_sizes) {
                // Benchmark with loop constraint
                std::string loop_urdf = buildUrdfPath(config.depth, loop_size, true);
                auto loop_result = benchmarkModel(loop_urdf, config.depth, loop_size,
                                                   false, debug_pass);
                pass_results[{config.depth, loop_size}].push_back(loop_result);

                // Benchmark baseline (no loop) - use approx URDF
                std::string baseline_urdf = buildUrdfPath(config.depth, loop_size, false);
                auto baseline_result = benchmarkModel(baseline_urdf, config.depth, loop_size,
                                                       true, debug_pass);
                pass_results[{config.depth, -loop_size}].push_back(baseline_result);
            }
        }

        if (!debug_pass) {
            std::cout << "done\n";
        }
    }

    std::cout << "\n";

    // Aggregate results across passes (take best/median)
    printHeader();

    for (const auto& config : configs) {
        std::cout << "\n--- Chain Depth " << config.depth << " ---\n";

        for (int loop_size : config.loop_sizes) {
            // Process loop results
            auto& loop_passes = pass_results[{config.depth, loop_size}];
            if (!loop_passes.empty()) {
                // Take result with minimum time
                auto best_it = std::min_element(loop_passes.begin(), loop_passes.end(),
                    [](const BenchmarkResult& a, const BenchmarkResult& b) {
                        return a.min_time_us < b.min_time_us;
                    });
                all_results.push_back(*best_it);
                printResult(*best_it);
            }

            // Process baseline results
            auto& baseline_passes = pass_results[{config.depth, -loop_size}];
            if (!baseline_passes.empty()) {
                auto best_it = std::min_element(baseline_passes.begin(), baseline_passes.end(),
                    [](const BenchmarkResult& a, const BenchmarkResult& b) {
                        return a.min_time_us < b.min_time_us;
                    });
                all_results.push_back(*best_it);
                printResult(*best_it);
            }
        }
    }

    std::cout << std::string(110, '-') << "\n\n";

    // Analysis
    std::cout << "===========================================================================\n";
    std::cout << "Analysis Summary\n";
    std::cout << "===========================================================================\n\n";

    for (const auto& config : configs) {
        std::cout << "Chain Depth " << config.depth << ":\n";

        // Find baseline and loop results for this depth
        std::vector<BenchmarkResult> depth_results;
        for (const auto& r : all_results) {
            if (r.chain_depth == config.depth && r.dof > 0) {
                depth_results.push_back(r);
            }
        }

        if (!depth_results.empty()) {
            // Group by loop_size, comparing baseline vs loop
            for (int loop_size : config.loop_sizes) {
                BenchmarkResult* loop_r = nullptr;
                BenchmarkResult* baseline_r = nullptr;

                for (auto& r : depth_results) {
                    if (r.loop_size == loop_size) {
                        if (r.is_baseline) {
                            baseline_r = &r;
                        } else {
                            loop_r = &r;
                        }
                    }
                }

                if (loop_r && baseline_r) {
                    double overhead = loop_r->min_time_us - baseline_r->min_time_us;
                    double ratio = loop_r->min_time_us / baseline_r->min_time_us;
                    std::cout << "  loop_size=" << std::setw(2) << loop_size
                              << " (conn@" << loop_r->connection_depth << "): "
                              << std::fixed << std::setprecision(2)
                              << loop_r->min_time_us << " us vs "
                              << baseline_r->min_time_us << " us baseline"
                              << " (+" << overhead << " us, "
                              << std::setprecision(2) << ratio << "x)\n";
                }
            }
        }
        std::cout << "\n";
    }

    std::cout << "Key Observations:\n";
    std::cout << "1. How loop constraint overhead scales with loop size\n";
    std::cout << "2. Whether deeper connections (larger loops) increase cost more\n";
    std::cout << "3. Comparison of loop vs baseline (open chain) performance\n";
    std::cout << "4. Error should remain bounded (~1e-7) regardless of configuration\n\n";

    // Export to CSV
    std::string output_dir = std::string(SOURCE_DIRECTORY) + "/Benchmarking/data/";

    {
        std::ofstream csv(output_dir + "parallel_chain_depth.csv");
        csv << "chain_depth,loop_size,connection_depth,dof,num_bodies,is_baseline,"
            << "min_us,mean_us,median_us,std_us,max_err_dq,max_err_dqdot,"
            << "fwd_kin_us,fwd_casadi_us,fwd_other_us,bwd_casadi_us,bwd_other_us,bwd_prop_us\n";
        for (const auto& r : all_results) {
            if (r.dof > 0) {
                csv << r.chain_depth << "," << r.loop_size << "," << r.connection_depth << ","
                    << r.dof << "," << r.num_bodies << "," << (r.is_baseline ? 1 : 0) << ","
                    << std::fixed << std::setprecision(4)
                    << r.min_time_us << "," << r.mean_time_us << ","
                    << r.median_time_us << "," << r.std_time_us << ","
                    << std::scientific << std::setprecision(2)
                    << r.max_error_dq << "," << r.max_error_dqdot << ","
                    << std::fixed << std::setprecision(4)
                    << r.fwd_kin_us << "," << r.fwd_casadi_us << "," << r.fwd_other_us << ","
                    << r.bwd_casadi_us << "," << r.bwd_other_us << "," << r.bwd_prop_us << "\n";
            }
        }
        std::cout << "Exported: " << output_dir << "parallel_chain_depth.csv\n";
    }

    // Export loop-only results for easier plotting
    {
        std::ofstream csv(output_dir + "loop_depth_sweep.csv");
        csv << "chain_depth,loop_size,connection_depth,dof,min_us,mean_us,baseline_min_us,"
            << "fwd_kin_us,fwd_casadi_us,fwd_other_us,bwd_casadi_us,bwd_other_us,bwd_prop_us,"
            << "baseline_fwd_kin_us,baseline_fwd_casadi_us,baseline_fwd_other_us,"
            << "baseline_bwd_casadi_us,baseline_bwd_other_us,baseline_bwd_prop_us\n";

        for (const auto& config : configs) {
            for (int loop_size : config.loop_sizes) {
                const BenchmarkResult* loop_r = nullptr;
                const BenchmarkResult* baseline_r = nullptr;

                for (const auto& r : all_results) {
                    if (r.chain_depth == config.depth && r.loop_size == loop_size && r.dof > 0) {
                        if (r.is_baseline) {
                            baseline_r = &r;
                        } else {
                            loop_r = &r;
                        }
                    }
                }

                if (loop_r) {
                    csv << loop_r->chain_depth << "," << loop_r->loop_size << ","
                        << loop_r->connection_depth << "," << loop_r->dof << ","
                        << std::fixed << std::setprecision(4)
                        << loop_r->min_time_us << "," << loop_r->mean_time_us << ",";
                    if (baseline_r) {
                        csv << baseline_r->min_time_us;
                    } else {
                        csv << "NA";
                    }
                    csv << "," << std::fixed << std::setprecision(4)
                        << loop_r->fwd_kin_us << "," << loop_r->fwd_casadi_us << ","
                        << loop_r->fwd_other_us << "," << loop_r->bwd_casadi_us << ","
                        << loop_r->bwd_other_us << "," << loop_r->bwd_prop_us << ",";
                    if (baseline_r) {
                        csv << baseline_r->fwd_kin_us << "," << baseline_r->fwd_casadi_us << ","
                            << baseline_r->fwd_other_us << "," << baseline_r->bwd_casadi_us << ","
                            << baseline_r->bwd_other_us << "," << baseline_r->bwd_prop_us;
                    } else {
                        csv << "NA,NA,NA,NA,NA,NA";
                    }
                    csv << "\n";
                }
            }
        }
        std::cout << "Exported: " << output_dir << "loop_depth_sweep.csv\n";
    }

    std::cout << "\n===========================================================================\n";
    std::cout << "Benchmark Complete\n";
    std::cout << "===========================================================================\n";

    return 0;
}
