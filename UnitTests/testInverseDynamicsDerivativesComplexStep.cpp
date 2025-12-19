#include <iostream>
#include <iomanip>
#include <complex>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

// Finite difference Jacobian helper
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

// Lie group configuration addition for complex-valued states
// Implements the retraction map: q_new = q ⊞ dq for floating bases with quaternions
template<typename T>
DVec<T> lieGroupConfigurationAddition(const DVec<T>& q0, const DVec<T>& dq, bool floating_base) {
    if (!floating_base) {
        // Simple vector space addition for fixed-base robots
        return q0 + dq;
    } else {
        // Lie group configuration addition for floating base with quaternions
        // q0 has size n_q (7 for floating base + n_joints) - configuration space
        // dq has size n_v (6 for floating base + n_joints) - velocity/tangent space
        const int n_q = q0.size();
        const int n_v = dq.size();
        const int nj = n_v - 6;  // Number of joint DOFs

        DVec<T> q_new = q0;

        // Joint DOFs use simple vector space addition
        q_new.tail(nj) += dq.tail(nj);

        // Extract current floating base configuration
        Eigen::Matrix<T, 4, 1> quat_vec = q0.head(4);        // Orientation quaternion [w, x, y, z]
        Eigen::Matrix<T, 3, 1> p = q0.segment(4, 3);         // Position in world frame

        // Update orientation using quaternion exponential map
        // For body frame angular velocity ω, the quaternion update is:
        //   q_new = q * exp(ω) where exp: so(3) → quaternion
        Eigen::Matrix<T, 3, 1> omega_body = dq.head(3);

        // Compute delta quaternion from angular velocity
        // exp(ω) = [cos(θ/2), sin(θ/2) * ω/θ] where θ = ||ω||
        T theta = omega_body.norm();
        Eigen::Matrix<T, 4, 1> delta_quat;

        if (std::real(theta) < 1e-10) {
            // Small angle approximation: exp(ω) ≈ [1, ω/2]
            delta_quat[0] = T(1.0);
            delta_quat.template tail<3>() = omega_body / T(2.0);
        } else {
            T half_theta = theta / T(2.0);
            delta_quat[0] = std::cos(half_theta);
            delta_quat.template tail<3>() = std::sin(half_theta) * omega_body / theta;
        }

        // Quaternion multiplication: q_new = q * delta_quat (right multiplication)
        Eigen::Matrix<T, 4, 1> quat_new;
        quat_new[0] = quat_vec[0] * delta_quat[0] - quat_vec.template tail<3>().dot(delta_quat.template tail<3>());
        quat_new.template tail<3>() = quat_vec[0] * delta_quat.template tail<3>() +
                                       delta_quat[0] * quat_vec.template tail<3>() +
                                       quat_vec.template tail<3>().cross(delta_quat.template tail<3>());

        // Normalize quaternion
        quat_new.normalize();

        // Update position: transform body-frame linear velocity to world frame
        // p_new = p + R^T * v_body where R = world-to-body rotation matrix
        // Quaternion to rotation matrix (world-to-body)
        T qw = quat_vec[0], qx = quat_vec[1], qy = quat_vec[2], qz = quat_vec[3];
        Eigen::Matrix<T, 3, 3> R;  // world-to-body
        R(0,0) = T(1) - T(2)*(qy*qy + qz*qz);
        R(0,1) = T(2)*(qx*qy + qw*qz);
        R(0,2) = T(2)*(qx*qz - qw*qy);
        R(1,0) = T(2)*(qx*qy - qw*qz);
        R(1,1) = T(1) - T(2)*(qx*qx + qz*qz);
        R(1,2) = T(2)*(qy*qz + qw*qx);
        R(2,0) = T(2)*(qx*qz + qw*qy);
        R(2,1) = T(2)*(qy*qz - qw*qx);
        R(2,2) = T(1) - T(2)*(qx*qx + qy*qy);

        Eigen::Matrix<T, 3, 1> v_body = dq.segment(3, 3);
        Eigen::Matrix<T, 3, 1> p_new = p + R.transpose() * v_body;  // R^T = body-to-world

        // Assemble new configuration
        q_new.head(4) = quat_new;
        q_new.segment(4, 3) = p_new;

        return q_new;
    }
}



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

// Helper function for testing with different Lie group configuration variants
// Parameters:
//   quat_order: 0 = [w,x,y,z] (default), 1 = [x,y,z,w]
//   use_R_transpose: true = use R^T, false = use R
void testInverseDynamicsDerivativesLieGroupVariant(ClusterTreeModel<double>& model,
                                                     const std::string& robot_name,
                                                     int expected_dof,
                                                     bool floating_base = false,
                                                     int quat_order = 0,
                                                     bool use_R_transpose = true,
                                                     double tol_dq = 1e-6,
                                                     double tol_dqdot = 1e-6) {
    std::cout << std::setprecision(12);

    const int nDOF = model.getNumDegreesOfFreedom();
    std::cout << "\n========================================\n";
    std::cout << "Testing inverse dynamics derivatives (Lie Group FD)\n";
    std::cout << "Robot: " << robot_name << "\n";
    std::cout << "DOF: " << nDOF << "\n";
    std::cout << "Quat order: " << (quat_order == 0 ? "[w,x,y,z]" : "[x,y,z,w]") << "\n";
    std::cout << "Position update: " << (use_R_transpose ? "R^T" : "R") << "\n";
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

    // Verify with finite differences using Lie group retraction
    std::pair<DVec<double>, DVec<double>> state = model.getState();
    const DVec<double>& q0 = state.first;
    const DVec<double>& qd0 = state.second;
    const double h = 1e-8;

    std::cout << "Finite difference verification (h = " << h << "):\n";
    std::cout << "  Tolerance: dtau/dq = " << tol_dq << ", dtau/dqdot = " << tol_dqdot << "\n\n";

    // Configuration addition with variants
    auto conf_add = [&](const DVec<double> &dq) -> DVec<double>
    {
        if(!floating_base) {
            return q0 + dq;
        }
        else {
            const int n_q = q0.size();
            const int n_v = dq.size();
            const int nj = n_v - 6;
            DVec<double> q_new = q0;
            q_new.tail(nj) += dq.tail(nj);

            // Extract quaternion (handle order)
            Quat<double> quat;
            if (quat_order == 0) {
                quat = q0.head(4);  // [w,x,y,z]
            } else {
                quat[0] = q0[3];  // w
                quat[1] = q0[0];  // x
                quat[2] = q0[1];  // y
                quat[3] = q0[2];  // z
            }
            Vec3<double> p = q0.segment(4, 3);

            Vec3<double> omega_body = dq.head(3);
            Quat<double> delta_quat = ori::so3ToQuat(omega_body);
            Quat<double> quat_new = ori::quatProduct(quat, delta_quat);
            quat_new.normalize();

            Mat3<double> R = ori::quaternionToRotationMatrix(quat);
            Vec3<double> v_body = dq.segment(3, 3);
            Vec3<double> p_new;
            if (use_R_transpose) {
                p_new = p + R.transpose() * v_body;
            } else {
                p_new = p + R * v_body;
            }

            // Store quaternion (handle order)
            if (quat_order == 0) {
                q_new.head(4) = quat_new;
            } else {
                q_new[0] = quat_new[1];  // x
                q_new[1] = quat_new[2];  // y
                q_new[2] = quat_new[3];  // z
                q_new[3] = quat_new[0];  // w
            }
            q_new.segment(4, 3) = p_new;

            return q_new;
        }
    };

    auto tau_func_q = [&](const DVec<double>& dq) {
        auto q = conf_add(dq);
        std::pair<DVec<double>, DVec<double>> state_q = {q, qd0};
        model.setState(state_q);
        return model.inverseDynamics(ydd);
    };

    auto tau_func_qd = [&](const DVec<double>& qd) {
        std::pair<DVec<double>, DVec<double>> state_qd = {q0, qd};
        model.setState(state_qd);
        return model.inverseDynamics(ydd);
    };

    auto dtau_dq_fd = finiteDifferenceJacobian(tau_func_q, qd0*0, h);
    auto dtau_dqdot_fd = finiteDifferenceJacobian(tau_func_qd, qd0, h);

    double max_error_dq = (dtau_dq - dtau_dq_fd).cwiseAbs().maxCoeff();
    double max_error_dqdot = (dtau_dqdot - dtau_dqdot_fd).cwiseAbs().maxCoeff();

    std::cout << "\n========================================\n";
    std::cout << "RESULTS:\n";
    std::cout << "  Max error (dtau/dq):    " << max_error_dq << " (tol: " << tol_dq << ")\n";
    std::cout << "  Max error (dtau/dqdot): " << max_error_dqdot << " (tol: " << tol_dqdot << ")\n";

    if (max_error_dq < tol_dq) {
        std::cout << "  dtau/dq: PASS ✓\n";
    } else {
        std::cout << "  dtau/dq: FAIL ✗\n";
    }

    if (max_error_dqdot < tol_dqdot) {
        std::cout << "  dtau/dqdot: PASS ✓\n";
    } else {
        std::cout << "  dtau/dqdot: FAIL ✗\n";
    }
    std::cout << "========================================\n\n";

    EXPECT_LT(max_error_dq, tol_dq);
    EXPECT_LT(max_error_dqdot, tol_dqdot);
}

TEST(InverseDynamicsDerivativesComplexStep, TwoLinkChain) {
    RevoluteChainWithAndWithoutRotor<0, 2> robot(true);
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesComplexStepSimple(model, "2-link revolute chain (random geometry)", 2);
}

// Case 1: Original quaternion [w,x,y,z] with R^T
TEST(InverseDynamicsDerivativesComplexStep, MiniCheetah_WXYZ_RT) {
    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesLieGroupVariant(model, "MiniCheetah [w,x,y,z] + R^T", 18,
                                                   true /*floating base*/,
                                                   0 /*quat_order: [w,x,y,z]*/,
                                                   true /*use R^T*/,
                                                   1e-6 /*tol_dq*/, 1e-6 /*tol_dqdot*/);
}

// Case 2: Original quaternion [w,x,y,z] with R
TEST(InverseDynamicsDerivativesComplexStep, MiniCheetah_WXYZ_R) {
    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesLieGroupVariant(model, "MiniCheetah [w,x,y,z] + R", 18,
                                                   true /*floating base*/,
                                                   0 /*quat_order: [w,x,y,z]*/,
                                                   false /*use R*/,
                                                   1e-6 /*tol_dq*/, 1e-6 /*tol_dqdot*/);
}

// Case 3: Swapped quaternion [x,y,z,w] with R^T
TEST(InverseDynamicsDerivativesComplexStep, MiniCheetah_XYZW_RT) {
    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesLieGroupVariant(model, "MiniCheetah [x,y,z,w] + R^T", 18,
                                                   true /*floating base*/,
                                                   1 /*quat_order: [x,y,z,w]*/,
                                                   true /*use R^T*/,
                                                   1e-6 /*tol_dq*/, 1e-6 /*tol_dqdot*/);
}

// Case 4: Swapped quaternion [x,y,z,w] with R
TEST(InverseDynamicsDerivativesComplexStep, MiniCheetah_XYZW_R) {
    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesLieGroupVariant(model, "MiniCheetah [x,y,z,w] + R", 18,
                                                   true /*floating base*/,
                                                   1 /*quat_order: [x,y,z,w]*/,
                                                   false /*use R*/,
                                                   1e-6 /*tol_dq*/, 1e-6 /*tol_dqdot*/);
}
