#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "grbda/Utils/IDDerivProfile.h"

using namespace grbda;

struct RobotBreakdown {
    std::string robot_name;
    std::string label;
    int dof = 0;
    double total_us = 0.0;
    double forward_us = 0.0;
    double backward_us = 0.0;
    double getsq_internal_us = 0.0;
    double casadi_s_us = 0.0;
    double casadi_s_ring_us = 0.0;
    double casadi_sdotqd_q_us = 0.0;
    double casadi_sdotqd_qd_us = 0.0;
    double setstate_total_us = 0.0;
    double setstate_casadi_s_us = 0.0;
    double setstate_casadi_s_ring_us = 0.0;
    double setstate_casadi_sdotqd_q_us = 0.0;
    double setstate_casadi_sdotqd_qd_us = 0.0;
};

int envIntOrDefault(const char *name, int fallback) {
    const char *raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    const int parsed = std::atoi(raw);
    return parsed > 0 ? parsed : fallback;
}

double medianOf(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

ModelState<double> setValidBenchmarkState(ClusterTreeModel<double> &model, const std::string &label) {
    const int nDOF = model.getNumDegreesOfFreedom();
    DVec<double> ydd_probe = DVec<double>::Random(nDOF);

    for (int attempt = 0; attempt < 512; ++attempt) {
        ModelState<double> model_state;
        model_state.reserve(model.clusters().size());

        bool joint_sampling_ok = true;
        for (const auto &cluster : model.clusters()) {
            try {
                model_state.push_back(cluster->joint_->randomJointState());
            } catch (const std::exception &) {
                joint_sampling_ok = false;
                break;
            }
        }

        if (!joint_sampling_ok) {
            continue;
        }

        try {
            model.setState(model_state);
            auto out = model.firstOrderInverseDynamicsDerivatives(ydd_probe);
            (void)out;
            return model_state;
        } catch (const std::exception &) {
            // Retry with a new sampled state.
        }
    }

    throw std::runtime_error("Failed to sample valid spanning state for breakdown profile: " + label);
}

RobotBreakdown benchmarkModel(ClusterTreeModel<double> &model,
                              const std::string &robot_name,
                              const std::string &label,
                              int trials,
                              int repeats,
                              int warmup_iters) {
    const ModelState<double> benchmark_state = setValidBenchmarkState(model, label);

    const int dof = model.getNumDegreesOfFreedom();
    DVec<double> ydd = DVec<double>::LinSpaced(dof, -0.25, 0.25);

    for (int i = 0; i < warmup_iters; ++i) {
        auto out = model.firstOrderInverseDynamicsDerivatives(ydd);
        (void)out;
    }

    std::vector<double> total_samples;
    std::vector<double> forward_samples;
    std::vector<double> backward_samples;
    std::vector<double> getsq_internal_samples;
    std::vector<double> casadi_s_samples;
    std::vector<double> casadi_s_ring_samples;
    std::vector<double> casadi_sdotqd_q_samples;
    std::vector<double> casadi_sdotqd_qd_samples;
    std::vector<double> setstate_total_samples;
    std::vector<double> setstate_casadi_s_samples;
    std::vector<double> setstate_casadi_s_ring_samples;
    std::vector<double> setstate_casadi_sdotqd_q_samples;
    std::vector<double> setstate_casadi_sdotqd_qd_samples;

    total_samples.reserve(static_cast<size_t>(repeats));
    forward_samples.reserve(static_cast<size_t>(repeats));
    backward_samples.reserve(static_cast<size_t>(repeats));
    getsq_internal_samples.reserve(static_cast<size_t>(repeats));
    casadi_s_samples.reserve(static_cast<size_t>(repeats));
    casadi_s_ring_samples.reserve(static_cast<size_t>(repeats));
    casadi_sdotqd_q_samples.reserve(static_cast<size_t>(repeats));
    casadi_sdotqd_qd_samples.reserve(static_cast<size_t>(repeats));
    setstate_total_samples.reserve(static_cast<size_t>(repeats));
    setstate_casadi_s_samples.reserve(static_cast<size_t>(repeats));
    setstate_casadi_s_ring_samples.reserve(static_cast<size_t>(repeats));
    setstate_casadi_sdotqd_q_samples.reserve(static_cast<size_t>(repeats));
    setstate_casadi_sdotqd_qd_samples.reserve(static_cast<size_t>(repeats));

    for (int i = 0; i < warmup_iters; ++i) {
        profiling::resetCurrentCall();
        model.setState(benchmark_state);
    }

    for (int rep = 0; rep < repeats; ++rep) {
        double setstate_casadi_s_acc = 0.0;
        double setstate_casadi_s_ring_acc = 0.0;
        double setstate_casadi_sdotqd_q_acc = 0.0;
        double setstate_casadi_sdotqd_qd_acc = 0.0;
        double forward_acc = 0.0;
        double backward_acc = 0.0;
        double getsq_internal_acc = 0.0;
        double casadi_s_acc = 0.0;
        double casadi_s_ring_acc = 0.0;
        double casadi_sdotqd_q_acc = 0.0;
        double casadi_sdotqd_qd_acc = 0.0;

        const auto t_state0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < trials; ++i) {
            profiling::resetCurrentCall();
            model.setState(benchmark_state);

            const auto call = profiling::getCurrentCall();
            setstate_casadi_s_acc += call.casadi_s_us;
            setstate_casadi_s_ring_acc += call.casadi_s_ring_us;
            setstate_casadi_sdotqd_q_acc += call.casadi_sdotqd_q_us;
            setstate_casadi_sdotqd_qd_acc += call.casadi_sdotqd_qd_us;
        }
        const auto t_state1 = std::chrono::high_resolution_clock::now();
        const double rep_setstate_total_us = std::chrono::duration<double, std::micro>(t_state1 - t_state0).count() /
                                             static_cast<double>(trials);

        const auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < trials; ++i) {
            auto out = model.firstOrderInverseDynamicsDerivatives(ydd);
            (void)out;

            const auto call = profiling::getCurrentCall();
            forward_acc += call.forward_us;
            backward_acc += call.backward_us;
            getsq_internal_acc += call.getsq_internal_us;
            casadi_s_acc += call.casadi_s_us;
            casadi_s_ring_acc += call.casadi_s_ring_us;
            casadi_sdotqd_q_acc += call.casadi_sdotqd_q_us;
            casadi_sdotqd_qd_acc += call.casadi_sdotqd_qd_us;
        }
        const auto t1 = std::chrono::high_resolution_clock::now();

        const double rep_total_us = std::chrono::duration<double, std::micro>(t1 - t0).count() /
                                    static_cast<double>(trials);

        setstate_total_samples.push_back(rep_setstate_total_us);
        setstate_casadi_s_samples.push_back(setstate_casadi_s_acc / static_cast<double>(trials));
        setstate_casadi_s_ring_samples.push_back(setstate_casadi_s_ring_acc / static_cast<double>(trials));
        setstate_casadi_sdotqd_q_samples.push_back(setstate_casadi_sdotqd_q_acc / static_cast<double>(trials));
        setstate_casadi_sdotqd_qd_samples.push_back(setstate_casadi_sdotqd_qd_acc / static_cast<double>(trials));
        total_samples.push_back(rep_total_us);
        forward_samples.push_back(forward_acc / static_cast<double>(trials));
        backward_samples.push_back(backward_acc / static_cast<double>(trials));
        getsq_internal_samples.push_back(getsq_internal_acc / static_cast<double>(trials));
        casadi_s_samples.push_back(casadi_s_acc / static_cast<double>(trials));
        casadi_s_ring_samples.push_back(casadi_s_ring_acc / static_cast<double>(trials));
        casadi_sdotqd_q_samples.push_back(casadi_sdotqd_q_acc / static_cast<double>(trials));
        casadi_sdotqd_qd_samples.push_back(casadi_sdotqd_qd_acc / static_cast<double>(trials));
    }

    RobotBreakdown out;
    out.robot_name = robot_name;
    out.label = label;
    out.dof = dof;
    out.total_us = medianOf(total_samples);
    out.forward_us = medianOf(forward_samples);
    out.backward_us = medianOf(backward_samples);
    out.getsq_internal_us = medianOf(getsq_internal_samples);
    out.casadi_s_us = medianOf(casadi_s_samples);
    out.casadi_s_ring_us = medianOf(casadi_s_ring_samples);
    out.casadi_sdotqd_q_us = medianOf(casadi_sdotqd_q_samples);
    out.casadi_sdotqd_qd_us = medianOf(casadi_sdotqd_qd_samples);
    out.setstate_total_us = medianOf(setstate_total_samples);
    out.setstate_casadi_s_us = medianOf(setstate_casadi_s_samples);
    out.setstate_casadi_s_ring_us = medianOf(setstate_casadi_s_ring_samples);
    out.setstate_casadi_sdotqd_q_us = medianOf(setstate_casadi_sdotqd_q_samples);
    out.setstate_casadi_sdotqd_qd_us = medianOf(setstate_casadi_sdotqd_qd_samples);
    return out;
}

template <typename RobotType>
RobotBreakdown benchmarkRobot(const std::string &robot_name,
                              const std::string &label,
                              int trials,
                              int repeats,
                              int warmup_iters) {
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    return benchmarkModel(model, robot_name, label, trials, repeats, warmup_iters);
}

RobotBreakdown benchmarkURDF(const std::string &urdf_path,
                             const std::string &robot_name,
                             const std::string &label,
                             int trials,
                             int repeats,
                             int warmup_iters) {
    ClusterTreeModel<double> model;
    model.buildModelFromURDF(urdf_path);
    return benchmarkModel(model, robot_name, label, trials, repeats, warmup_iters);
}

void printSummary(const std::vector<RobotBreakdown> &rows) {
    std::cout << "\n" << std::string(120, '=') << "\n";
    std::cout << "ID Derivatives Breakdown (c6, clang -march=native)\n";
    std::cout << std::string(120, '=') << "\n\n";

    std::cout << std::left << std::setw(30) << "Robot"
              << std::right << std::setw(8) << "DOF"
              << std::setw(14) << "Total"
              << std::setw(14) << "Forward"
              << std::setw(14) << "Backward"
              << std::setw(14) << "getSq"
              << std::setw(14) << "Other"
              << std::setw(14) << "CasadiS"
              << std::setw(14) << "CasadiSr"
              << std::setw(14) << "CasSdotq"
              << std::setw(14) << "CasSdotqd"
              << "\n";

    std::cout << std::string(120, '-') << "\n";

    for (const auto &r : rows) {
        const double other_us = r.total_us - r.forward_us - r.backward_us - r.getsq_internal_us;

        std::cout << std::left << std::setw(30) << r.label
                  << std::right << std::setw(8) << r.dof
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.total_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.forward_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.backward_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.getsq_internal_us
                  << std::setw(14) << std::fixed << std::setprecision(2) << other_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.casadi_s_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.casadi_s_ring_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.casadi_sdotqd_q_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.casadi_sdotqd_qd_us
                  << "\n";
    }

    std::cout << std::string(120, '-') << "\n";
}

void printSetStateSummary(const std::vector<RobotBreakdown> &rows) {
    std::cout << "\n" << std::string(108, '=') << "\n";
    std::cout << "State-Update CasADi Breakdown (setState path)\n";
    std::cout << std::string(108, '=') << "\n\n";

    std::cout << std::left << std::setw(30) << "Robot"
              << std::right << std::setw(8) << "DOF"
              << std::setw(14) << "setState"
              << std::setw(14) << "CasadiS"
              << std::setw(14) << "CasadiSr"
              << std::setw(14) << "CasSdotq"
              << std::setw(14) << "CasSdotqd"
              << "\n";

    std::cout << std::string(108, '-') << "\n";

    for (const auto &r : rows) {
        std::cout << std::left << std::setw(30) << r.label
                  << std::right << std::setw(8) << r.dof
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.setstate_total_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.setstate_casadi_s_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.setstate_casadi_s_ring_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.setstate_casadi_sdotqd_q_us
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.setstate_casadi_sdotqd_qd_us
                  << "\n";
    }

    std::cout << std::string(108, '-') << "\n";
}

void writeCsv(const std::vector<RobotBreakdown> &rows, const std::string &path) {
    std::ofstream out(path);
    if (!out.is_open()) {
        std::cerr << "[WARN] Could not write CSV to: " << path << "\n";
        return;
    }

    out << "robot_name,label,dof,total_us,forward_us,backward_us,getsq_internal_us,other_us,casadi_s_us,casadi_s_ring_us,casadi_sdotqd_q_us,casadi_sdotqd_qd_us,setstate_total_us,setstate_casadi_s_us,setstate_casadi_s_ring_us,setstate_casadi_sdotqd_q_us,setstate_casadi_sdotqd_qd_us\n";
    for (const auto &r : rows) {
        const double other_us = r.total_us - r.forward_us - r.backward_us - r.getsq_internal_us;
        out << r.robot_name << ","
            << r.label << ","
            << r.dof << ","
            << std::fixed << std::setprecision(6) << r.total_us << ","
            << std::fixed << std::setprecision(6) << r.forward_us << ","
            << std::fixed << std::setprecision(6) << r.backward_us << ","
            << std::fixed << std::setprecision(6) << r.getsq_internal_us << ","
            << std::fixed << std::setprecision(6) << other_us << ","
            << std::fixed << std::setprecision(6) << r.casadi_s_us << ","
            << std::fixed << std::setprecision(6) << r.casadi_s_ring_us << ","
            << std::fixed << std::setprecision(6) << r.casadi_sdotqd_q_us << ","
            << std::fixed << std::setprecision(6) << r.casadi_sdotqd_qd_us << ","
            << std::fixed << std::setprecision(6) << r.setstate_total_us << ","
            << std::fixed << std::setprecision(6) << r.setstate_casadi_s_us << ","
            << std::fixed << std::setprecision(6) << r.setstate_casadi_s_ring_us << ","
            << std::fixed << std::setprecision(6) << r.setstate_casadi_sdotqd_q_us << ","
            << std::fixed << std::setprecision(6) << r.setstate_casadi_sdotqd_qd_us << "\n";
    }

    std::cout << "[INFO] Wrote CSV: " << path << "\n";
}

int main() {
    const int trials = envIntOrDefault("GRBDA_BREAKDOWN_TRIALS", 120);
    const int repeats = envIntOrDefault("GRBDA_BREAKDOWN_REPEATS", 9);
    const int warmup_iters = envIntOrDefault("GRBDA_BREAKDOWN_WARMUP", 40);

    const std::string urdf_path = std::string(SOURCE_DIRECTORY) + "/robot-models";

    std::vector<RobotBreakdown> rows;

    auto tryRun = [&](const std::string &friendly_name, auto fn) {
        try {
            std::cout << "Benchmarking " << friendly_name << "..." << std::flush;
            rows.push_back(fn());
            std::cout << " done\n";
        } catch (const std::exception &e) {
            std::cout << " failed\n";
            std::cerr << "[WARN] " << friendly_name << " skipped: " << e.what() << "\n";
        }
    };

    tryRun("KUKA LWR 4+", [&]() {
        return benchmarkURDF(urdf_path + "/kuka_lwr_4plus.urdf",
                             "KUKA_LWR_4plus", "KUKA LWR 4+ (-R)",
                             trials, repeats, warmup_iters);
    });

    tryRun("MiniCheetah (+R)", [&]() {
        return benchmarkRobot<MiniCheetah<double, ori_representation::Quaternion>>(
            "MiniCheetah_rotors", "Mini Cheetah (+R)",
            trials, repeats, warmup_iters);
    });

    tryRun("MiniCheetah (-R)", [&]() {
        return benchmarkURDF(urdf_path + "/mini_cheetah_approximate.urdf",
                             "MiniCheetah_no_rotors", "Mini Cheetah (-R)",
                             trials, repeats, warmup_iters);
    });

    tryRun("MIT Humanoid (+R)", [&]() {
        return benchmarkRobot<MIT_Humanoid<double, ori_representation::Quaternion>>(
            "MIT_Humanoid_rotors", "MIT Humanoid (+R)",
            trials, repeats, warmup_iters);
    });

    tryRun("MIT Humanoid (-R)", [&]() {
        return benchmarkRobot<MIT_Humanoid_no_rotors<double, ori_representation::Quaternion>>(
            "MIT_Humanoid_no_rotors", "MIT Humanoid (-R)",
            trials, repeats, warmup_iters);
    });

    tryRun("Tello (-R/-M)", [&]() {
        return benchmarkRobot<TelloNoRotors<double>>(
            "Tello_no_rotors_no_mech", "Tello (-R/-M)",
            trials, repeats, warmup_iters);
    });

    tryRun("Tello (+R/-M)", [&]() {
        return benchmarkRobot<TelloRotorsNoConstraints<double>>(
            "Tello_rotors_no_mech", "Tello (+R/-M)",
            trials, repeats, warmup_iters);
    });

    tryRun("Tello (+R/+M)", [&]() {
        return benchmarkRobot<Tello<double>>(
            "Tello_rotors_mech", "Tello (+R/+M)",
            trials, repeats, warmup_iters);
    });

    tryRun("TelloWithArms", [&]() {
        return benchmarkRobot<TelloWithArms<double>>(
            "TelloWithArms", "Tello with Arms (+R/+M)",
            trials, repeats, warmup_iters);
    });

    printSummary(rows);
    printSetStateSummary(rows);

    std::string csv_path = std::string(SOURCE_DIRECTORY) +
                           "/../benchmark_figures/data/fig4_performance_breakdown_c6.csv";
    const char *override_csv = std::getenv("GRBDA_BREAKDOWN_CSV");
    if (override_csv != nullptr && override_csv[0] != '\0') {
        csv_path = override_csv;
    }

    writeCsv(rows, csv_path);

    return rows.empty() ? 1 : 0;
}
