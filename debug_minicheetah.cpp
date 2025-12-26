#include <iostream>
#include <iomanip>
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

int main() {
    std::cout << std::setprecision(12);

    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();

    const int nDOF = model.getNumDegreesOfFreedom();
    std::cout << "DOF: " << nDOF << "\n\n";

    // Set random state
    ModelState<double> model_state;
    for (const auto &cluster : model.clusters()) {
        JointState<> joint_state = cluster->joint_->randomJointState();
        model_state.push_back(joint_state);
    }
    model.setState(model_state);

    // Get state
    std::pair<DVec<double>, DVec<double>> state = model.getState();
    const DVec<double>& q0 = state.first;
    const DVec<double>& qd0 = state.second;

    std::cout << "Configuration space dimension: " << q0.size() << "\n";
    std::cout << "Velocity space dimension: " << qd0.size() << "\n\n";

    std::cout << "First few config values:\n";
    for(int i = 0; i < std::min(10, (int)q0.size()); i++) {
        std::cout << "  q[" << i << "] = " << q0[i] << "\n";
    }

    std::cout << "\nQuaternion (q[3:6]): [" << q0[3] << ", " << q0[4] << ", " << q0[5] << ", " << q0[6] << "]\n";
    std::cout << "Quaternion norm: " << q0.segment(3,4).norm() << "\n";

    return 0;
}
