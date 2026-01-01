#include <iostream>
#include <iomanip>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/ClusterJoints/RevolutePairAbsoluteJoint.h"

using namespace grbda;

TEST(RevolutePairAbsoluteDerivativesComplexStep, ComplexStepValidation) {
    using Scalar = double;
    
    std::cout << std::setprecision(12);
    std::cout << "\n========================================\n";
    std::cout << "Testing RevolutePairAbsolute derivatives (Complex-Step)\n";
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
    std::cout << "  dtau_dq:    " << dtau_dq.rows() << " x " << dtau_dq.cols() << "\n";
    std::cout << "  dtau_dqdot: " << dtau_dqdot.rows() << " x " << dtau_dqdot.cols() << "\n\n";
    
    // Complex-step verification
    auto state_pair = model.getState();
    const DVec<Scalar>& q0 = state_pair.first;
    const DVec<Scalar>& qd0 = state_pair.second;
    
    const int nDOF = 2;
    const double h = 1e-20;  // Complex-step size
    
    std::cout << "Complex-step verification (h = " << h << "):" << std::endl;
    std::cout << "  Tolerance: dtau/dq = 1e-12, dtau/dqdot = 1e-12\n\n";
    
    // Build complex-step model
    ClusterTreeModel<std::complex<double>> model_complex{};
    
    const auto inertia_c = SpatialInertia<std::complex<double>>(
        std::complex<double>(1.0), 
        Vec3<std::complex<double>>::Zero(), 
        Mat3<std::complex<double>>::Identity());
    
    const auto X1_c = spatial::Transform<std::complex<double>>();
    const Vec3<std::complex<double>> r2_c(0.5, 0.0, 0.0);
    const auto X2_c = spatial::Transform<std::complex<double>>(
        Mat3<std::complex<double>>::Identity(), r2_c);
    
    auto body1_c = model_complex.registerBody("body1", inertia_c, "ground", X1_c);
    auto body2_c = model_complex.registerBody("body2", inertia_c, "ground", X2_c);
    
    const Vec3<std::complex<double>> r_internal_c(0.5, 0.0, 0.0);
    const auto X_internal_c = spatial::Transform<std::complex<double>>(
        Mat3<std::complex<double>>::Identity(), r_internal_c);
    
    model_complex.template appendRegisteredBodiesAsCluster<
        ClusterJoints::RevolutePairAbsolute<std::complex<double>>>(
        "cluster0", axis1, axis2, X_internal_c);
    
    // Verify derivatives using complex-step
    DMat<Scalar> dtau_dq_cs = DMat<Scalar>::Zero(nDOF, nDOF);
    DMat<Scalar> dtau_dqdot_cs = DMat<Scalar>::Zero(nDOF, nDOF);
    
    // Test dtau/dq
    for (int i = 0; i < nDOF; ++i) {
        DVec<std::complex<double>> q_complex = q0.cast<std::complex<double>>();
        q_complex(i) += std::complex<double>(0.0, h);
        
        DVec<std::complex<double>> qd_complex = qd0.cast<std::complex<double>>();
        DVec<std::complex<double>> qdd_complex = qdd.cast<std::complex<double>>();
        
        std::pair<DVec<std::complex<double>>, DVec<std::complex<double>>> state_c = 
            {q_complex, qd_complex};
        model_complex.setState(state_c);
        
        DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(qdd_complex);
        
        for (int j = 0; j < nDOF; ++j) {
            dtau_dq_cs(j, i) = tau_complex(j).imag() / h;
        }
        
        double error = (dtau_dq.col(i) - dtau_dq_cs.col(i)).cwiseAbs().maxCoeff();
        std::cout << "  dtau/dq" << i << " error: " << error;
        if (error < 1e-12) {
            std::cout << " [PASS]" << std::endl;
        } else {
            std::cout << " [FAIL]" << std::endl;
        }
    }
    
    std::cout << std::endl;
    
    // Test dtau/dqdot
    for (int i = 0; i < nDOF; ++i) {
        DVec<std::complex<double>> q_complex = q0.cast<std::complex<double>>();
        DVec<std::complex<double>> qd_complex = qd0.cast<std::complex<double>>();
        qd_complex(i) += std::complex<double>(0.0, h);
        
        DVec<std::complex<double>> qdd_complex = qdd.cast<std::complex<double>>();
        
        std::pair<DVec<std::complex<double>>, DVec<std::complex<double>>> state_c = 
            {q_complex, qd_complex};
        model_complex.setState(state_c);
        
        DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(qdd_complex);
        
        for (int j = 0; j < nDOF; ++j) {
            dtau_dqdot_cs(j, i) = tau_complex(j).imag() / h;
        }
        
        double error = (dtau_dqdot.col(i) - dtau_dqdot_cs.col(i)).cwiseAbs().maxCoeff();
        std::cout << "  dtau/dqd" << i << " error: " << error;
        if (error < 1e-12) {
            std::cout << " [PASS]" << std::endl;
        } else {
            std::cout << " [FAIL]" << std::endl;
        }
    }
    
    // Check errors
    double error_dq = (dtau_dq - dtau_dq_cs).cwiseAbs().maxCoeff();
    double error_dqdot = (dtau_dqdot - dtau_dqdot_cs).cwiseAbs().maxCoeff();
    
    std::cout << std::endl;
    std::cout << "========================================\n";
    std::cout << "RESULTS:\n";
    std::cout << "  Max error (dtau/dq):    " << error_dq << " (tol: 1e-12)\n";
    std::cout << "  Max error (dtau/dqdot): " << error_dqdot << " (tol: 1e-12)\n";
    std::cout << "========================================\n\n";
    
    const double tol = 1e-12;
    EXPECT_LT(error_dq, tol) << "dtau/dq error exceeds tolerance";
    EXPECT_LT(error_dqdot, tol) << "dtau/dqdot error exceeds tolerance";
}
