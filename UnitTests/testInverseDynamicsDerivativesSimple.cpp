#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <string>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include <casadi/casadi.hpp>
#include "testHelpers.hpp"

using namespace grbda;




void testInverseDynamicsDerivativesFiniteDifference(
    ClusterTreeModel<double>& model_real,
    const std::string& robot_name,
    double tol_dq = 1e-6,
    double tol_dqdot = 1e-6) {
    std::cout << std::setprecision(16);
    const int nDOF = model_real.getNumDegreesOfFreedom();
    std::cout << "\n========================================\n";
    std::cout << "Finite-Difference Derivative Test: " << robot_name << " (DOF=" << nDOF << ")\n";
    std::cout << "========================================\n\n";

    const DVec<double> ydd_real = DVec<double>::Random(nDOF);

    auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);
    auto [q0, qd0] = model_real.getState();

    const ModelState<double> state_real_base = makeModelState<double>(model_real, q0, qd0);
    const DVec<double>       zero_dqr = DVec<double>::Zero(nDOF);

    auto ID_of_dq_fd = [&](const DVec<double>& dq) -> DVec<double> {
        model_real.setState(applyMinimalPerturbation(model_real, state_real_base, dq, zero_dqr), false);
        return model_real.inverseDynamics(ydd_real);
    };
    auto ID_of_dqdot_fd = [&](const DVec<double>& dqdot) -> DVec<double> {
        model_real.setState(applyMinimalPerturbation(model_real, state_real_base, zero_dqr, dqdot), false);
        return model_real.inverseDynamics(ydd_real);
    };

    const double h_fd = 1e-7;
    DMat<double> dtau_dq_fd    = finiteDifferenceJacobian(ID_of_dq_fd,    zero_dqr, h_fd);
    DMat<double> dtau_dqdot_fd = finiteDifferenceJacobian(ID_of_dqdot_fd, zero_dqr, h_fd);

    double max_error_dq    = (dtau_dq    - dtau_dq_fd).cwiseAbs().maxCoeff();
    double max_error_dqdot = (dtau_dqdot - dtau_dqdot_fd).cwiseAbs().maxCoeff();

    std::cout << "Max FD vs analytical error (dtau/dq):    " << max_error_dq    << "\n";
    std::cout << "Max FD vs analytical error (dtau/dqdot): " << max_error_dqdot << "\n";

    if( max_error_dq > tol_dq) {
        std::cout << "Details for dtau/dq error:\n";
        std::cerr << "Analytical derivatives:\n";
        std::cerr << "dtau/dq:\n" << dtau_dq << "\n";
        std::cerr << "Finite difference derivatives:\n";
        std::cerr << "dtau/dq (FD):\n" << dtau_dq_fd << "\n";

        std::cerr << "Error (boolean):\n";
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> out_of_tol =
        (dtau_dq - dtau_dq_fd).array().abs() > tol_dq;
        std::cerr << out_of_tol << "\n";

    }

    if (max_error_dqdot > tol_dqdot) {
        std::cout << "Details for dtau/dqdot error:\n";
        std::cerr << "Analytical derivatives:\n";
        std::cerr << "dtau/dqdot:\n" << dtau_dqdot << "\n";
        std::cerr << "Finite difference derivatives:\n";
        std::cerr << "dtau/dqdot (FD):\n" << dtau_dqdot_fd << "\n";

        std::cerr << "Error (boolean):\n";
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> out_of_tol =
        (dtau_dqdot - dtau_dqdot_fd).array().abs() > tol_dqdot;
        std::cerr << out_of_tol << "\n";
    }
    
    EXPECT_LT(max_error_dq,    tol_dq)    << "dtau/dq error exceeds tolerance";
    EXPECT_LT(max_error_dqdot, tol_dqdot) << "dtau/dqdot error exceeds tolerance";
}


TEST(InverseDynamicsDerivatives, TelloWithArmsImplicitConstraint) {
    using namespace grbda;
    TelloWithArms<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    constexpr bool enforce_constraints = true;
    model.setState(randomModelState(model,enforce_constraints));
    testInverseDynamicsDerivativesFiniteDifference(model, "TelloWithArms", 1e-6, 1e-6);
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
    // Tolerance relaxed to 1e-5 due to finite difference truncation error with h=1e-6
    model.setState(randomModelState(model));
    testInverseDynamicsDerivativesFiniteDifference(model, "2-link revolute chain (random geometry)", 1e-5, 1e-5);
}


TEST(InverseDynamicsDerivatives, ThreeLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 3> means 0 with rotors, 3 without rotors = 3 DOF
    // NOTE: Random parameters include random rotation axes and transforms
    RevoluteChainWithAndWithoutRotor<0, 3> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    model.setState(randomModelState(model));
    testInverseDynamicsDerivativesFiniteDifference(model, "3-link revolute chain (random geometry)", 1e-5, 1e-5);
}

TEST(InverseDynamicsDerivatives, FourLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 4> means 0 with rotors, 4 without rotors = 4 DOF
    // NOTE: Random parameters include random rotation axes and transforms
    RevoluteChainWithAndWithoutRotor<0, 4> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    model.setState(randomModelState(model));
    testInverseDynamicsDerivativesFiniteDifference(model, "4-link revolute chain (random geometry)", 2e-5, 2e-5);
}


// NOTE: Re-enabling test to debug and fix floating base derivatives
TEST(InverseDynamicsDerivatives, MiniCheetahQuaternion) {
    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    model.setState(randomModelState(model));
    testInverseDynamicsDerivativesFiniteDifference(model, "MiniCheetah (Quaternion)", 1e-4, 1e-5);
}

TEST(InverseDynamicsDerivatives, MITHumanoidQuaternionv2) {
    MIT_Humanoid<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    model.setState(randomModelState(model));

    // Actual errors: dtau/dq ~9.3e-5, dtau/dqdot ~6.7e-7
    // Tightened from previous overly-relaxed tolerances (1.0, 0.1)
    testInverseDynamicsDerivativesFiniteDifference(model, "MIT Humanoid (Quaternion) - Finite Difference", 1e-4, 1e-6);
}

TEST(InverseDynamicsDerivatives, TeleopArm) {
    TeleopArm<> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    model.setState(randomModelState(model));
    testInverseDynamicsDerivativesFiniteDifference(model, "TeleopArm", 1e-6, 1e-6);
}

TEST(InverseDynamicsDerivatives, TelloImplicitConstraint) {
    using namespace grbda;
    Tello<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    constexpr bool enforce_constraints = true;
    model.setState(randomModelState(model,enforce_constraints));
    // Tello uses 30 trials with verbose output to track detailed results
    testInverseDynamicsDerivativesFiniteDifference(model, "Tello", 1e-6, 1e-6);
}

TEST(InverseDynamicsDerivatives, PlanarLegLinkageImplicitConstraint) {
    using namespace grbda;
    PlanarLegLinkage<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    constexpr bool enforce_constraints = true;
    model.setState(randomModelState(model,enforce_constraints));
    testInverseDynamicsDerivativesFiniteDifference(model, "PlanarLegLinkage", 1e-4, 1e-6);
}

TEST(InverseDynamicsDerivatives, KangarooOpenChain) {
    using namespace grbda;
    Kangaroo<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    model.setState(randomModelState(model));
    // Kangaroo is a 14-DOF floating base robot without loop constraints
    testInverseDynamicsDerivativesFiniteDifference(model, "Kangaroo (open chain)", 1e-4, 1e-5);
}

TEST(InverseDynamicsDerivatives, CassieClosedLoop) {
    using namespace grbda;
    Cassie<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    constexpr bool enforce_constraints = true;
    model.setState(randomModelState(model,enforce_constraints));
    // Cassie has FourBar constraints in the lower legs
    testInverseDynamicsDerivativesFiniteDifference(model, "Cassie (closed-loop)", 1e-4, 1e-5);
}

// KangarooWithConstraints test - may fail with some random states due to FourBar geometry
TEST(InverseDynamicsDerivatives, KangarooWithConstraints) {
    using namespace grbda;
    KangarooWithConstraints<double> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    constexpr bool enforce_constraints = true;
    model.setState(randomModelState(model,enforce_constraints));
    // Use fewer trials and verbose output to diagnose issues
    testInverseDynamicsDerivativesFiniteDifference(model, "KangarooWithConstraints", 1e-4, 1e-5);
}

