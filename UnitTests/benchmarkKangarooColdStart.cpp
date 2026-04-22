#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

struct ModelResult {
    std::string name;
    int dof;
    int bodies;
    double build_ms;
    double fk_ms;
    double id_ms;
    double total_ms;
};

template<typename RobotType>
ModelResult profileModel(const std::string& name) {
    ModelResult result;
    result.name = name;

    auto t_start = std::chrono::high_resolution_clock::now();
    RobotType robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    auto t_build = std::chrono::high_resolution_clock::now();

    result.dof = model.getNumDegreesOfFreedom();
    result.bodies = model.getNumBodies();
    result.build_ms = std::chrono::duration<double, std::milli>(t_build - t_start).count();

    ModelState<double> state;
    bool state_set = false;
    for (int retry = 0; retry < 100 && !state_set; ++retry) {
        try {
            state.clear();
            for (const auto& cluster : model.clusters()) {
                JointState<double> joint_state = cluster->joint_->randomJointState();
                state.push_back(joint_state);
            }
            model.setState(state);
            state_set = true;
        } catch (const std::exception&) {
        }
    }

    if (!state_set) {
        state.clear();
        for (const auto& cluster : model.clusters()) {
            JointState<double> joint_state(cluster->joint_->isImplicit(), false);
            joint_state.position = DVec<double>::Zero(cluster->joint_->numPositions());
            joint_state.velocity = DVec<double>::Zero(cluster->joint_->numVelocities());

            if (!cluster->joint_->isImplicit() && cluster->joint_->numPositions() == 7) {
                joint_state.position(3) = 1.0;
            }

            state.push_back(joint_state);
        }
        model.setState(state);
    }

    auto t_fk_start = std::chrono::high_resolution_clock::now();
    model.forwardKinematics();
    auto t_fk_end = std::chrono::high_resolution_clock::now();
    result.fk_ms = std::chrono::duration<double, std::milli>(t_fk_end - t_fk_start).count();

    DVec<double> qdd = DVec<double>::Zero(result.dof);
    auto t_id_start = std::chrono::high_resolution_clock::now();
    model.inverseDynamics(qdd);
    auto t_id_end = std::chrono::high_resolution_clock::now();
    result.id_ms = std::chrono::duration<double, std::milli>(t_id_end - t_id_start).count();

    result.total_ms = result.build_ms + result.fk_ms + result.id_ms;
    return result;
}

void printResult(const ModelResult& r) {
    std::cout << std::left << std::setw(50) << r.name
              << std::right << std::setw(5) << r.dof
              << std::setw(8) << r.bodies
              << std::setw(12) << std::fixed << std::setprecision(3) << r.build_ms
              << std::setw(12) << r.fk_ms
              << std::setw(12) << r.id_ms
              << std::setw(12) << r.total_ms << "\n";
}

int main() {
    std::cout << "\n" << std::string(105, '=') << "\n";
    std::cout << "KANGAROO ROBOT COLD START BENCHMARK\n";
    std::cout << std::string(105, '=') << "\n\n";

    std::vector<ModelResult> results;

    std::cout << "Profiling models...\n\n";

    std::cout << "  [1/6] Kangaroo (open chain)..." << std::flush;
    results.push_back(profileModel<Kangaroo<double>>("Kangaroo [open chain]"));
    std::cout << " done\n";

    std::cout << "  [2/6] Kangaroo (with 4-bar constraint)..." << std::flush;
    results.push_back(profileModel<KangarooWithConstraints<double>>("Kangaroo [4-bar knee constraint]"));
    std::cout << " done\n";

    std::cout << "  [3/6] Cassie (closed-loop leg)..." << std::flush;
    results.push_back(profileModel<Cassie<double>>("Cassie [closed-loop leg]"));
    std::cout << " done\n";

    std::cout << "  [4/6] Tello (baseline)..." << std::flush;
    results.push_back(profileModel<TelloNoRotors<double>>("Tello [baseline]"));
    std::cout << " done\n";

    std::cout << "  [5/6] MIT Humanoid..." << std::flush;
    results.push_back(profileModel<MIT_Humanoid<double>>("MIT_Humanoid [serial chain]"));
    std::cout << " done\n";

    std::cout << "  [6/6] MIT Humanoid Leg..." << std::flush;
    results.push_back(profileModel<MIT_Humanoid_Leg<double>>("MIT_Humanoid_Leg [belt transmissions]"));
    std::cout << " done\n";

    std::cout << "\n" << std::string(105, '=') << "\n";
    std::cout << "RESULTS\n";
    std::cout << std::string(105, '-') << "\n";
    std::cout << std::left << std::setw(50) << "Model"
              << std::right << std::setw(5) << "DOF"
              << std::setw(8) << "Bodies"
              << std::setw(12) << "Build(ms)"
              << std::setw(12) << "FK(ms)"
              << std::setw(12) << "ID(ms)"
              << std::setw(12) << "Total(ms)" << "\n";
    std::cout << std::string(105, '-') << "\n";

    for (const auto& r : results) {
        printResult(r);
    }

    std::cout << std::string(105, '=') << "\n";
    std::cout << "\nNOTES:\n";
    std::cout << "- Kangaroo: PAL Robotics biped with serial-parallel hybrid legs\n";
    std::cout << "- Kangaroo 4-bar: includes knee mechanism with FourBar constraint\n";
    std::cout << "- Cassie: Agility Robotics biped with closed-loop lower leg\n";
    std::cout << "- Sources: PAL Robotics, Gepetto example-parallel-robots\n";
    std::cout << std::string(105, '=') << "\n";

    return 0;
}
