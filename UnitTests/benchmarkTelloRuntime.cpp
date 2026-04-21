#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "grbda/Utils/IDDerivProfile.h"

using namespace grbda;

struct SummaryStats {
    double mean_us = 0.0;
    double median_us = 0.0;
    double p95_us = 0.0;
    double min_us = 0.0;
    double max_us = 0.0;
};

struct BreakdownStats {
    double forward_us = 0.0;
    double backward_us = 0.0;
    double casadi_us = 0.0;
    double getsq_us = 0.0;
    double getsq_internal_us = 0.0;
};

struct OperationRow {
    std::string label;
    SummaryStats stats;
};

int envIntOrDefault(const char *name, int fallback) {
    const char *raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    const int parsed = std::atoi(raw);
    return parsed > 0 ? parsed : fallback;
}

bool envFlagEnabled(const char *name, bool fallback) {
    const char *raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    if (raw[0] == '0') {
        return false;
    }
    return true;
}

SummaryStats computeStatsUs(const std::vector<double> &samples_us) {
    SummaryStats out;
    if (samples_us.empty()) {
        return out;
    }

    std::vector<double> sorted = samples_us;
    std::sort(sorted.begin(), sorted.end());

    const size_t n = sorted.size();
    const size_t med_idx = n / 2;
    const size_t p95_idx = static_cast<size_t>(0.95 * static_cast<double>(n - 1));

    out.min_us = sorted.front();
    out.max_us = sorted.back();
    out.median_us = sorted[med_idx];
    out.p95_us = sorted[p95_idx];
    out.mean_us = std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(n);
    return out;
}

template <typename Callable>
SummaryStats measureOperationUs(int repeats, Callable &&operation) {
    std::vector<double> samples_us;
    samples_us.reserve(static_cast<size_t>(repeats));

    for (int r = 0; r < repeats; ++r) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        operation();
        const auto t1 = std::chrono::high_resolution_clock::now();
        samples_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }

    return computeStatsUs(samples_us);
}

template <typename RobotType>
void profileRuntime(const std::string &name, int trials, int repeats, bool include_breakdown_pass) {
    std::cout << "\n" << std::string(90, '=') << "\n";
    std::cout << "RUNTIME PROFILE: " << name << "\n";
    std::cout << std::string(90, '=') << "\n";

    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();

    const int dof = model.getNumDegreesOfFreedom();
    std::cout << "DOF: " << dof << ", Bodies: " << model.getNumBodies() << "\n\n";

    DVec<double> qdd = DVec<double>::LinSpaced(dof, -0.25, 0.25);

    bool valid_state_found = false;
    for (int attempt = 0; attempt < 512; ++attempt) {
        ModelState<double> state;
        state.reserve(model.clusters().size());

        bool sample_ok = true;
        for (const auto &cluster : model.clusters()) {
            try {
                state.push_back(cluster->joint_->randomJointState());
            } catch (const std::exception &) {
                sample_ok = false;
                break;
            }
        }
        if (!sample_ok) {
            continue;
        }

        try {
            model.setState(state);
            model.forwardKinematics();
            model.inverseDynamics(qdd);
            model.firstOrderInverseDynamicsDerivatives(qdd);
            valid_state_found = true;
            break;
        } catch (const std::exception &) {
            // Retry on invalid spanning positions/constraints.
        }
    }

    if (!valid_state_found) {
        throw std::runtime_error("Failed to sample valid spanning state for runtime profile: " + name);
    }

    model.forwardKinematics();
    model.inverseDynamics(qdd);
    model.firstOrderInverseDynamicsDerivatives(qdd);

    auto fk_stats = measureOperationUs(repeats, [&]() {
        for (int i = 0; i < trials; ++i) {
            model.forwardKinematics();
        }
    });
    fk_stats.mean_us /= static_cast<double>(trials);
    fk_stats.median_us /= static_cast<double>(trials);
    fk_stats.p95_us /= static_cast<double>(trials);
    fk_stats.min_us /= static_cast<double>(trials);
    fk_stats.max_us /= static_cast<double>(trials);

    auto id_stats = measureOperationUs(repeats, [&]() {
        for (int i = 0; i < trials; ++i) {
            model.inverseDynamics(qdd);
        }
    });
    id_stats.mean_us /= static_cast<double>(trials);
    id_stats.median_us /= static_cast<double>(trials);
    id_stats.p95_us /= static_cast<double>(trials);
    id_stats.min_us /= static_cast<double>(trials);
    id_stats.max_us /= static_cast<double>(trials);

    auto deriv_stats = measureOperationUs(repeats, [&]() {
        for (int i = 0; i < trials; ++i) {
            model.firstOrderInverseDynamicsDerivatives(qdd);
        }
    });
    deriv_stats.mean_us /= static_cast<double>(trials);
    deriv_stats.median_us /= static_cast<double>(trials);
    deriv_stats.p95_us /= static_cast<double>(trials);
    deriv_stats.min_us /= static_cast<double>(trials);
    deriv_stats.max_us /= static_cast<double>(trials);

    std::vector<OperationRow> rows = {
        {"Forward Kinematics", fk_stats},
        {"Inverse Dynamics", id_stats},
        {"ID Derivatives", deriv_stats},
    };

    std::cout << "PRODUCTION TIMING (no profile reads in timed loop)\n";
    std::cout << "trials per repeat=" << trials << ", repeats=" << repeats << "\n\n";

    std::cout << std::left << std::setw(30) << "Operation"
              << std::right << std::setw(14) << "mean (us)"
              << std::setw(14) << "median"
              << std::setw(14) << "p95"
              << std::setw(14) << "min"
              << std::setw(14) << "max"
              << std::setw(14) << "Hz@median"
              << "\n";
    std::cout << std::string(114, '-') << "\n";

    for (const auto &row : rows) {
        const double hz = row.stats.median_us > 0.0 ? (1000000.0 / row.stats.median_us) : 0.0;
        std::cout << std::left << std::setw(30) << row.label
                  << std::right << std::setw(14) << std::fixed << std::setprecision(2) << row.stats.mean_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << row.stats.median_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << row.stats.p95_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << row.stats.min_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << row.stats.max_us
                  << std::setw(14) << std::fixed << std::setprecision(1) << hz
                  << "\n";
    }

    const double loop_median = fk_stats.median_us + id_stats.median_us + deriv_stats.median_us;
    const double loop_hz = loop_median > 0.0 ? (1000000.0 / loop_median) : 0.0;

    std::cout << std::string(114, '-') << "\n";
    std::cout << std::left << std::setw(30) << "FULL CONTROL LOOP"
              << std::right << std::setw(14) << std::fixed << std::setprecision(2)
              << (fk_stats.mean_us + id_stats.mean_us + deriv_stats.mean_us)
              << std::setw(14) << std::fixed << std::setprecision(2) << loop_median
              << std::setw(14) << std::fixed << std::setprecision(2)
              << (fk_stats.p95_us + id_stats.p95_us + deriv_stats.p95_us)
              << std::setw(14) << std::fixed << std::setprecision(2)
              << (fk_stats.min_us + id_stats.min_us + deriv_stats.min_us)
              << std::setw(14) << std::fixed << std::setprecision(2)
              << (fk_stats.max_us + id_stats.max_us + deriv_stats.max_us)
              << std::setw(14) << std::fixed << std::setprecision(1) << loop_hz
              << "\n";

    if (!include_breakdown_pass || name.find("Tello (+R,+M)") == std::string::npos) {
        std::cout << std::string(90, '=') << "\n";
        return;
    }

    BreakdownStats breakdown;
    for (int i = 0; i < trials; ++i) {
        model.firstOrderInverseDynamicsDerivatives(qdd);
        const auto call = profiling::getCurrentCall();
        breakdown.forward_us += call.forward_us;
        breakdown.backward_us += call.backward_us;
        breakdown.casadi_us += call.casadi_us;
        breakdown.getsq_us += call.getsq_us;
        breakdown.getsq_internal_us += call.getsq_internal_us;
    }
    breakdown.forward_us /= static_cast<double>(trials);
    breakdown.backward_us /= static_cast<double>(trials);
    breakdown.casadi_us /= static_cast<double>(trials);
    breakdown.getsq_us /= static_cast<double>(trials);
    breakdown.getsq_internal_us /= static_cast<double>(trials);

    std::cout << "\nDIAGNOSTIC BREAKDOWN PASS (not part of production timing)\n";
    std::cout << std::left << std::setw(34) << "Stage"
              << std::right << std::setw(14) << "avg (us)"
              << "\n";
    std::cout << std::string(48, '-') << "\n";
    std::cout << std::left << std::setw(34) << "Profile forward"
              << std::right << std::setw(14) << std::fixed << std::setprecision(2) << breakdown.forward_us << "\n";
    std::cout << std::left << std::setw(34) << "Profile backward"
              << std::right << std::setw(14) << std::fixed << std::setprecision(2) << breakdown.backward_us << "\n";
    std::cout << std::left << std::setw(34) << "Profile CasADi"
              << std::right << std::setw(14) << std::fixed << std::setprecision(2) << breakdown.casadi_us << "\n";
    std::cout << std::left << std::setw(34) << "Profile getSq (outer)"
              << std::right << std::setw(14) << std::fixed << std::setprecision(2) << breakdown.getsq_us << "\n";
    std::cout << std::left << std::setw(34) << "Profile getSq (internal)"
              << std::right << std::setw(14) << std::fixed << std::setprecision(2) << breakdown.getsq_internal_us << "\n";

    std::cout << std::string(90, '=') << "\n";
}

int main() {
    const int trials = envIntOrDefault("GRBDA_RUNTIME_TRIALS", 100);
    const int repeats = envIntOrDefault("GRBDA_RUNTIME_REPEATS", 7);
    const bool include_breakdown_pass = envFlagEnabled("GRBDA_RUNTIME_INCLUDE_BREAKDOWN", true);

    std::cout << "\n" << std::string(90, '=') << "\n";
    std::cout << "TELLO ROBOT RUNTIME PERFORMANCE BENCHMARK\n";
    std::cout << std::string(90, '=') << "\n";
    std::cout << "Production timing mode separates clean runtime from diagnostics.\n";

    profileRuntime<TelloNoRotors<double>>("Tello (-R,-M) [baseline]", trials, repeats, false);
    profileRuntime<TelloRotorsNoConstraints<double>>("Tello (+R,-M) [rotors only]", trials, repeats, false);
    profileRuntime<TelloMechanismsNoRotorsStatic<double>>("Tello (-R,+M-Static) [linear constraints]", trials, repeats, false);
    profileRuntime<Tello<double>>("Tello (+R,+M) [full model with LazyGenericImplicit]", trials, repeats, include_breakdown_pass);
    profileRuntime<TelloWithArms<double>>("TelloWithArms (+R,+M)", trials, repeats, false);

    std::cout << "\nKEY INSIGHT:\n";
    std::cout << "- Production rows are the numbers to compare across commits\n";
    std::cout << "- Diagnostic breakdown is for attribution only\n";
    std::cout << std::string(90, '=') << "\n";

    return 0;
}
