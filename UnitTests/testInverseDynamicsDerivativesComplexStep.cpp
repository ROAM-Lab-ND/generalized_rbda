#include <iostream>
#include <iomanip>
#include <complex>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

// NOTE: This test uses complex-step differentiation to verify inverse dynamics derivatives.
// Complex-step provides machine-precision derivatives without subtractive cancellation errors.
// The method computes: f'(x) ≈ Im(f(x + ih)) / h  where i is the imaginary unit.

// Helper function to convert real state to complex state
std::pair<DVec<std::complex<double>>, DVec<std::complex<double>>>
toComplexState(const DVec<double>& q, const DVec<double>& qd) {
    DVec<std::complex<double>> q_complex(q.size());
    DVec<std::complex<double>> qd_complex(qd.size());

    for (int i = 0; i < q.size(); ++i) {
        q_complex[i] = std::complex<double>(q[i], 0.0);
        qd_complex[i] = std::complex<double>(qd[i], 0.0);
    }

    return {q_complex, qd_complex};
}

// Helper function to run complex-step derivative test on simple serial chain models
// NOTE: This version only works for models with simple revolute joints (no rotors, no free joints)
void testInverseDynamicsDerivativesComplexStepSimple(ClusterTreeModel<double>& model_real,
                                                       const std::string& robot_name,
                                                       int expected_dof,
                                                       double tol_dq = 1e-12,
                                                       double tol_dqdot = 1e-12) {
    std::cout << std::setprecision(16);

    const int nDOF = model_real.getNumDegreesOfFreedom();
    std::cout << "\n========================================\n";
    std::cout << "Testing inverse dynamics derivatives (Complex-Step)\n";
    std::cout << "Robot: " << robot_name << "\n";
    std::cout << "DOF: " << nDOF << "\n";
    std::cout << "========================================\n\n";

    ASSERT_EQ(nDOF, expected_dof);

    // Set random state on real model
    ModelState<double> model_state_real;
    for (const auto &cluster : model_real.clusters()) {
        JointState<> joint_state = cluster->joint_->randomJointState();
        model_state_real.push_back(joint_state);
    }
    model_real.setState(model_state_real);

    // Random acceleration
    const DVec<double> ydd_real = DVec<double>::Random(nDOF);

    // Get analytical derivatives
    auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);

    std::cout << "Analytical derivatives computed successfully.\n";
    std::cout << "  dtau_dq:    " << dtau_dq.rows() << " x " << dtau_dq.cols() << "\n";
    std::cout << "  dtau_dqdot: " << dtau_dqdot.rows() << " x " << dtau_dqdot.cols() << "\n\n";

    // Get real state
    std::pair<DVec<double>, DVec<double>> state_real = model_real.getState();
    const DVec<double>& q0 = state_real.first;
    const DVec<double>& qd0 = state_real.second;

    // Create complex model
    ClusterTreeModel<std::complex<double>> model_complex;

    // Copy structure from real model
    using namespace ClusterJoints;

    // Rebuild the model with complex types (simple revolute joints only)
    for (size_t i = 0; i < model_real.bodies().size(); ++i) {
        const auto& body = model_real.bodies()[i];

        // Convert spatial inertia to complex
        SpatialInertia<std::complex<double>> inertia_c(
            std::complex<double>(body.inertia_.getMass(), 0.0),
            body.inertia_.getCOM().template cast<std::complex<double>>(),
            body.inertia_.getInertiaTensor().template cast<std::complex<double>>()
        );

        // Convert transform to complex
        spatial::Transform<std::complex<double>> Xtree_c(
            body.Xtree_.getRotation().template cast<std::complex<double>>(),
            body.Xtree_.getTranslation().template cast<std::complex<double>>()
        );

        // Find parent name
        std::string parent_name = "ground";
        if (body.parent_index_ >= 0 && body.parent_index_ < model_real.bodies().size()) {
            parent_name = model_real.bodies()[body.parent_index_].name_;
        }

        Body<std::complex<double>> body_c = model_complex.registerBody(
            body.name_, inertia_c, parent_name, Xtree_c
        );

        // Get the joint axis from the real cluster
        if (i < model_real.clusters().size()) {
            auto cluster = model_real.cluster(i);
            const DMat<double>& S = cluster->S();

            // Determine the joint axis from the motion subspace matrix
            // For a revolute joint, S = [w; 0] where w is the rotation axis
            ori::CoordinateAxis axis;
            if (std::abs(S(0)) > 0.9) {
                axis = ori::CoordinateAxis::X;
            } else if (std::abs(S(1)) > 0.9) {
                axis = ori::CoordinateAxis::Y;
            } else if (std::abs(S(2)) > 0.9) {
                axis = ori::CoordinateAxis::Z;
            } else {
                throw std::runtime_error("Complex-step test only supports axis-aligned revolute joints");
            }

            model_complex.appendRegisteredBodiesAsCluster<Revolute<std::complex<double>>>(
                body.name_, body_c, axis, body.name_ + "_joint"
            );
        }
    }

    const double h = 1e-20;  // Step size for complex-step (can be very small)
    const std::complex<double> ih(0.0, h);

    std::cout << "Complex-step verification (h = " << h << "):\n";
    std::cout << "  Tolerance: dtau/dq = " << tol_dq << ", dtau/dqdot = " << tol_dqdot << "\n\n";

    // Convert ydd to complex
    DVec<std::complex<double>> ydd_complex(nDOF);
    for (int i = 0; i < nDOF; ++i) {
        ydd_complex[i] = std::complex<double>(ydd_real[i], 0.0);
    }

    // Test dtau/dq using complex-step
    double max_error_dq = 0.0;
    for (int i = 0; i < nDOF; ++i) {
        // Create perturbed state: q[i] += ih
        auto [q_complex, qd_complex] = toComplexState(q0, qd0);
        q_complex[i] += ih;

        // Convert to ModelState
        ModelState<std::complex<double>> model_state_complex;
        int idx = 0;
        for (const auto &cluster : model_complex.clusters()) {
            JointCoordinate<std::complex<double>> pos(
                DVec<std::complex<double>>::Zero(cluster->num_positions_), false);
            JointCoordinate<std::complex<double>> vel(
                DVec<std::complex<double>>::Zero(cluster->num_velocities_), false);

            for (int j = 0; j < cluster->num_positions_; ++j) {
                pos[j] = q_complex[idx + j];
            }
            for (int j = 0; j < cluster->num_velocities_; ++j) {
                vel[j] = qd_complex[idx + j];
            }

            JointState<std::complex<double>> joint_state(pos, vel);
            model_state_complex.push_back(joint_state);
            idx += cluster->num_velocities_;
        }

        model_complex.setState(model_state_complex);
        DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

        // Extract derivative from imaginary part
        DVec<double> dtau_dqi_cs(nDOF);
        for (int j = 0; j < nDOF; ++j) {
            dtau_dqi_cs[j] = tau_complex[j].imag() / h;
        }

        double error = (dtau_dqi_cs - dtau_dq.col(i)).norm();
        max_error_dq = std::max(max_error_dq, error);

        std::cout << "  dtau/dq" << i << " error: " << error;
        if (error < tol_dq) std::cout << " [PASS]";
        else std::cout << " [FAIL]";
        std::cout << "\n";

        EXPECT_LT(error, tol_dq);
    }

    std::cout << "\n";

    // Test dtau/dqdot using complex-step
    double max_error_dqdot = 0.0;
    for (int i = 0; i < nDOF; ++i) {
        // Create perturbed state: qd[i] += ih
        auto [q_complex, qd_complex] = toComplexState(q0, qd0);
        qd_complex[i] += ih;

        // Convert to ModelState
        ModelState<std::complex<double>> model_state_complex;
        int idx = 0;
        for (const auto &cluster : model_complex.clusters()) {
            JointCoordinate<std::complex<double>> pos(
                DVec<std::complex<double>>::Zero(cluster->num_positions_), false);
            JointCoordinate<std::complex<double>> vel(
                DVec<std::complex<double>>::Zero(cluster->num_velocities_), false);

            for (int j = 0; j < cluster->num_positions_; ++j) {
                pos[j] = q_complex[idx + j];
            }
            for (int j = 0; j < cluster->num_velocities_; ++j) {
                vel[j] = qd_complex[idx + j];
            }

            JointState<std::complex<double>> joint_state(pos, vel);
            model_state_complex.push_back(joint_state);
            idx += cluster->num_velocities_;
        }

        model_complex.setState(model_state_complex);
        DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

        // Extract derivative from imaginary part
        DVec<double> dtau_dqdoti_cs(nDOF);
        for (int j = 0; j < nDOF; ++j) {
            dtau_dqdoti_cs[j] = tau_complex[j].imag() / h;
        }

        double error = (dtau_dqdoti_cs - dtau_dqdot.col(i)).norm();
        max_error_dqdot = std::max(max_error_dqdot, error);

        std::cout << "  dtau/dqd" << i << " error: " << error;
        if (error < tol_dqdot) std::cout << " [PASS]";
        else std::cout << " [FAIL]";
        std::cout << "\n";

        EXPECT_LT(error, tol_dqdot);
    }

    std::cout << "\n========================================\n";
    std::cout << "RESULTS:\n";
    std::cout << "  Max error (dtau/dq):    " << max_error_dq << " (tol: " << tol_dq << ")\n";
    std::cout << "  Max error (dtau/dqdot): " << max_error_dqdot << " (tol: " << tol_dqdot << ")\n";
    std::cout << "========================================\n\n";
}

TEST(InverseDynamicsDerivativesComplexStep, DoublePendulumURDF) {
    ClusterTreeModel<double> model;
    model.buildModelFromURDF("/home/docker/generalized_rbda/robot-models/double_pendulum.urdf");
    // 2-link double pendulum from URDF should work perfectly with complex-step
    testInverseDynamicsDerivativesComplexStepSimple(model, "Double pendulum (URDF)", 2);
}

TEST(InverseDynamicsDerivativesComplexStep, ThreeLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 3> means 0 with rotors, 3 without rotors = 3 DOF
    // NOTE: Random parameters include random rotation axes and transforms,
    //       making the geometry much more complex than simple Z-axis chains
    RevoluteChainWithAndWithoutRotor<0, 3> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesComplexStepSimple(model, "3-link revolute chain (random geometry)", 3);
}

TEST(InverseDynamicsDerivativesComplexStep, FourLinkChain) {
    // RevoluteChainWithAndWithoutRotor<N, M> where N=rotors, M=no rotors
    // So <0, 4> means 0 with rotors, 4 without rotors = 4 DOF
    // NOTE: Random parameters include random rotation axes and transforms,
    //       making the geometry much more complex than simple Z-axis chains
    RevoluteChainWithAndWithoutRotor<0, 4> robot(true); // use random parameters
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesComplexStepSimple(model, "4-link revolute chain (random geometry)", 4);
}

// NOTE: MiniCheetah tests are not included in the complex-step test because:
// 1. The complex model reconstruction code assumes simple revolute joints with Z-axis
// 2. MiniCheetah has RevoluteWithRotor joints and multiple joint axes
// 3. Rebuilding the complex structure would require significant refactoring
// MiniCheetah is tested with finite differences in testInverseDynamicsDerivativesSimple.cpp
