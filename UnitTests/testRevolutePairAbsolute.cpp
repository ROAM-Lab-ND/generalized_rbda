#include <iostream>
#include <iomanip>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/ClusterJoints/RevolutePairAbsoluteJoint.h"

using namespace grbda;

TEST(RevolutePairAbsoluteDerivatives, FiniteDifferenceValidation) {
    using Scalar = double;
    
    std::cout << std::setprecision(12);
    std::cout << "\n========================================\n";
    std::cout << "Testing RevolutePairAbsolute derivatives\n";
    std::cout << "========================================\n\n";
    
    // Create a simple 2-DOF model with one RevolutePairAbsolute joint
    ClusterTreeModel<Scalar> model{};
    
    // Register two bodies
    const auto inertia = SpatialInertia<Scalar>(1.0, Vec3<Scalar>::Zero(), 
                                                Mat3<Scalar>::Identity());
    
    // Identity transform
    const auto X1 = spatial::Transform<Scalar>();
    
    // Translation transform (0.5m in X)
    const Vec3<Scalar> r2(0.5, 0.0, 0.0);
    const auto X2 = spatial::Transform<Scalar>(Mat3<Scalar>::Identity(), r2);
    
    auto body1 = model.registerBody("body1", inertia, "ground", X1);
    auto body2 = model.registerBody("body2", inertia, "ground", X2);
    
    // Create cluster with RevolutePairAbsolute joint
    ori::CoordinateAxis axis1 = ori::CoordinateAxis::Z;
    ori::CoordinateAxis axis2 = ori::CoordinateAxis::Z;
    
    const Vec3<Scalar> r_internal(0.5, 0.0, 0.0);
    const auto X_internal = spatial::Transform<Scalar>(Mat3<Scalar>::Identity(), r_internal);
    
    model.template appendRegisteredBodiesAsCluster<ClusterJoints::RevolutePairAbsolute<Scalar>>(
        "cluster0", axis1, axis2, X_internal);
    
    ASSERT_EQ(model.getNumDegreesOfFreedom(), 2);
    
    // Set test state
    JointState<Scalar> state;
    state.position = DVec<Scalar>(2);
    state.velocity = DVec<Scalar>(2);
    state.position << 0.3, 0.8;
    state.velocity << 0.1, 0.2;
    
    ModelState<Scalar> model_state;
    model_state.push_back(state);
    model.setState(model_state);
    
    // Test acceleration
    DVec<Scalar> qdd = DVec<Scalar>::Random(2);
    
    // Get analytical derivatives
    auto [dtau_dq, dtau_dqdot] = model.firstOrderInverseDynamicsDerivatives(qdd);
    
    std::cout << "Analytical derivatives computed.\n";
    std::cout << "  dtau_dq:\n" << dtau_dq << "\n\n";
    std::cout << "  dtau_dqdot:\n" << dtau_dqdot << "\n\n";
    
    // Finite difference verification
    auto state_pair = model.getState();
    const DVec<Scalar>& q0 = state_pair.first;
    const DVec<Scalar>& qd0 = state_pair.second;
    const double h = 1e-8;
    
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
    
    auto tau_func_q = [&](const DVec<Scalar>& q) {
        std::pair<DVec<Scalar>, DVec<Scalar>> s = {q, qd0};
        model.setState(s);
        return model.inverseDynamics(qdd);
    };
    
    auto tau_func_qd = [&](const DVec<Scalar>& qd) {
        std::pair<DVec<Scalar>, DVec<Scalar>> s = {q0, qd};
        model.setState(s);
        return model.inverseDynamics(qdd);
    };
    
    auto dtau_dq_fd = finiteDifferenceJacobian(tau_func_q, q0, h);
    auto dtau_dqdot_fd = finiteDifferenceJacobian(tau_func_qd, qd0, h);
    
    std::cout << "Finite difference derivatives:\n";
    std::cout << "  dtau_dq (FD):\n" << dtau_dq_fd << "\n\n";
    std::cout << "  dtau_dqdot (FD):\n" << dtau_dqdot_fd << "\n\n";
    
    // Check errors
    double error_dq = (dtau_dq - dtau_dq_fd).cwiseAbs().maxCoeff();
    double error_dqdot = (dtau_dqdot - dtau_dqdot_fd).cwiseAbs().maxCoeff();
    
    std::cout << "========================================\n";
    std::cout << "RESULTS:\n";
    std::cout << "  Max error (dtau/dq):    " << error_dq << "\n";
    std::cout << "  Max error (dtau/dqdot): " << error_dqdot << "\n";
    std::cout << "========================================\n\n";
    
    const double tol = 1e-5;
    EXPECT_LT(error_dq, tol) << "dtau/dq error exceeds tolerance";
    EXPECT_LT(error_dqdot, tol) << "dtau/dqdot error exceeds tolerance";
}
