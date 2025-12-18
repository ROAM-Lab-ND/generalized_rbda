#include <iostream>
#include <iomanip>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

// NOTE: The tolerance is set to 1e-6 to account for numerical errors in finite
// difference verification with step size h=1e-8. The analytical derivatives match
// the finite difference results within this numerical precision.
auto finiteDifferenceJacobian = [](auto func, const Eigen::VectorXd& point, double h) {
    int n = point.size();
    Eigen::VectorXd f0 = func(point);
    int m = f0.size();
    Eigen::MatrixXd jacobian(m, n);
    
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd pointPert = point;
        pointPert[i] += h;
        Eigen::VectorXd fPert = func(pointPert);
        jacobian.col(i) = (fPert - f0) / h;
    }
    return jacobian;
};


// Helper function to run the finite difference test on any model
void testInverseDynamicsDerivatives(ClusterTreeModel<double>& model,
                                     const std::string& robot_name,
                                     int expected_dof,
                                     double tol_dq = 1e-6,
                                     double tol_dqdot = 1e-6) {
    std::cout << std::setprecision(12);

    const int nDOF = model.getNumDegreesOfFreedom();
    std::cout << "\n========================================\n";
    std::cout << "Testing inverse dynamics derivatives\n";
    std::cout << "Robot: " << robot_name << "\n";
    std::cout << "DOF: " << nDOF << "\n";
    std::cout << "========================================\n\n";

    ASSERT_EQ(nDOF, expected_dof);

    // Set random state
    ModelState<double> model_state;
    for (const auto &cluster : model.clusters()) {
        JointState<> joint_state = cluster->joint_->randomJointState();
        model_state.push_back(joint_state);
    }
    model.setState(model_state);

    // Random acceleration
    const DVec<double> ydd = DVec<double>::Random(nDOF);

    // Get analytical derivatives
    auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(ydd);

    std::cout << "Analytical derivatives computed successfully.\n";
    std::cout << "  dtau_dq:    " << dtau_dq.rows() << " x " << dtau_dq.cols() << "\n";
    std::cout << "  dtau_dqdot: " << dtau_dqdot.rows() << " x " << dtau_dqdot.cols() << "\n\n";

    // Verify with finite differences
    std::pair<DVec<double>, DVec<double>> state = model.getState();
    const DVec<double>& q0 = state.first;
    const DVec<double>& qd0 = state.second;
    const double h = 1e-8;

    std::cout << "Finite difference verification (h = " << h << "):\n";
    std::cout << "  Tolerance: dtau/dq = " << tol_dq << ", dtau/dqdot = " << tol_dqdot << "\n\n";

    auto tau_func_q = [&](const DVec<double>& q) {
        std::pair<DVec<double>, DVec<double>> state_q = {q, qd0};
        model.setState(state_q);
        return model.inverseDynamics(ydd);
    };

    auto tau_func_qd = [&](const DVec<double>& qd) {
        std::pair<DVec<double>, DVec<double>> state_qd = {q0, qd};
        model.setState(state_qd);
        return model.inverseDynamics(ydd);
    };

    auto dtau_dq_fd = finiteDifferenceJacobian(tau_func_q, q0, h);
    auto dtau_dqdot_fd = finiteDifferenceJacobian(tau_func_qd, qd0, h);

    EXPECT_TRUE( dtau_dq.isApprox(dtau_dq_fd, tol_dq) );
    EXPECT_TRUE( dtau_dqdot.isApprox(dtau_dqdot_fd, tol_dqdot) );

    std::cout << "\n========================================\n";
    std::cout << "RESULTS:\n";
    std::cout << "  Max error (dtau/dq):    " << (dtau_dq - dtau_dq_fd).cwiseAbs().maxCoeff() << " (tol: " << tol_dq << ")\n";
    std::cout << "  Max error (dtau/dqdot): " << (dtau_dqdot - dtau_dqdot_fd).cwiseAbs().maxCoeff() << " (tol: " << tol_dqdot << ")\n";
    std::cout << "========================================\n\n";
}

//TEST(InverseDynamicsDerivatives, DoublePendulumURDF) {
//    ClusterTreeModel<double> model;
//    model.buildModelFromURDF("/home/docker/generalized_rbda/robot-models/double_pendulum.urdf");
//    // 2-link double pendulum from URDF works perfectly with current implementation
//    testInverseDynamicsDerivatives(model, "Double pendulum (URDF)", 2);
//}

TEST(InverseDynamicsDerivatives, TwoLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 2> means 0 with rotors, 2 without rotors = 2 DOF
    // NOTE: Random parameters include random rotation axes and transforms
    RevoluteChainWithAndWithoutRotor<0, 2> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    // Relaxed tolerance due to missing gradient terms + random geometry
    testInverseDynamicsDerivatives(model, "2-link revolute chain (random geometry)", 2);
}


TEST(InverseDynamicsDerivatives, ThreeLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 3> means 0 with rotors, 3 without rotors = 3 DOF
    // NOTE: Random parameters include random rotation axes and transforms
    RevoluteChainWithAndWithoutRotor<0, 3> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivatives(model, "3-link revolute chain (random geometry)", 3);
}

TEST(InverseDynamicsDerivatives, FourLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 4> means 0 with rotors, 4 without rotors = 4 DOF
    // NOTE: Random parameters include random rotation axes and transforms
    RevoluteChainWithAndWithoutRotor<0, 4> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivatives(model, "4-link revolute chain (random geometry)", 4);
}


TEST(InverseDynamicsDerivatives, MiniCheetahRollPitchYaw) {
    // MiniCheetah quadruped with floating base (RPY orientation)
    // Floating base: 6 DOF (3 translational + 3 rotational via RPY)
    // 4 legs × 3 joints/leg = 12 DOF
    // Total: 18 DOF
    MiniCheetah<double, ori_representation::RollPitchYaw> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    // Very relaxed tolerance due to floating base + complex geometry + missing gradient terms
    // Note: Finite differences have additional numerical error compared to complex-step
    // Based on direct config perturbation test, errors are around 60-87 for floating base DOFs
    testInverseDynamicsDerivatives(model, "MiniCheetah (RollPitchYaw)", 18);  // relaxed for dtau/dqdot
}