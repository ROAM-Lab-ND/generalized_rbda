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

// Helper to normalize quaternions only for real types (not complex)
// Uses SFINAE to avoid breaking complex-step differentiation
template<typename Derived>
typename std::enable_if<std::is_arithmetic<typename Derived::Scalar>::value, void>::type
normalizeQuaternionIfReal(Eigen::MatrixBase<Derived>& quat) {
    // Normalize for real types (double, float)
    const_cast<Eigen::MatrixBase<Derived>&>(quat).normalize();
}

template<typename Derived>
typename std::enable_if<!std::is_arithmetic<typename Derived::Scalar>::value, void>::type
normalizeQuaternionIfReal(Eigen::MatrixBase<Derived>& quat) {
    // Do NOT normalize for complex types - this preserves the imaginary part
    // which is essential for complex-step differentiation
}

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
        // NOTE: Configuration ordering is [pos(3), quat(4)] based on Joint.h Free joint
        // NOTE: q0 should already have a normalized quaternion (normalized before conversion to complex)
        Eigen::Matrix<T, 3, 1> p = q0.head(3);              // Position in world frame
        Eigen::Matrix<T, 4, 1> quat_vec = q0.segment(3, 4);  // Orientation quaternion [w, x, y, z]

        // Update orientation using quaternion exponential map
        // For body frame angular velocity ω, the quaternion update is:
        //   q_new = q * exp(ω) where exp: so(3) → quaternion
        Eigen::Matrix<T, 3, 1> omega_body = dq.head(3);

        // Compute delta quaternion from angular velocity
        // KEY INSIGHT FROM MATLAB: For complex-step, use LINEAR approximation!
        // exp(ω) = [cos(θ/2), sin(θ/2) * ω/θ] where θ = ||ω||
        // But for complex ω, use first-order: [1, ω/2]
        Eigen::Matrix<T, 4, 1> delta_quat;

        // CRITICAL FIX: Check if the VALUES have non-zero imaginary part (runtime check)
        // This matches MATLAB's approach: if ~isreal(dq{i})
        // NOT a compile-time type check!
        bool has_imag = false;
        if constexpr (!std::is_arithmetic<T>::value) {
            // For complex types, check if imaginary part is non-zero
            for (int i = 0; i < 3; ++i) {
                if (std::abs(std::imag(omega_body[i])) > 1e-30) {
                    has_imag = true;
                    break;
                }
            }
        }

        // DEBUG: Print what path we're taking
        static bool first_call = true;
        if (first_call && !std::is_arithmetic<T>::value) {
            std::cout << "DEBUG lieGroupConfigurationAddition:\n";
            std::cout << "  omega_body = " << omega_body.transpose() << "\n";
            std::cout << "  has_imag = " << has_imag << "\n";
            std::cout << "  imag(omega[0]) = " << std::imag(omega_body[0]) << "\n";
            std::cout << "  imag(omega[1]) = " << std::imag(omega_body[1]) << "\n";
            std::cout << "  imag(omega[2]) = " << std::imag(omega_body[2]) << "\n";
            first_call = false;
        }

        if (has_imag) {
            // COMPLEX-STEP: Use tangent directly (not exponential)
            // tang = [0, ω/2] (not exp([0, ω/2]) = [1, ω/2])
            delta_quat[0] = T(0.0);
            delta_quat.template tail<3>() = omega_body / T(2.0);
        } else {
            // REAL: Use full exponential map
            T theta = omega_body.norm();
            if (std::abs(theta) < 1e-10) {
                delta_quat[0] = T(1.0);
                delta_quat.template tail<3>() = omega_body / T(2.0);
            } else {
                T half_theta = theta / T(2.0);
                delta_quat[0] = std::cos(half_theta);
                delta_quat.template tail<3>() = std::sin(half_theta) * omega_body / theta;
            }
        }

        // Quaternion update: q_new = q * delta_quat (right multiplication)
        // For complex-step, use matrix form: q_new = (I + quatR(delta_quat)) * q
        Eigen::Matrix<T, 4, 1> quat_new;

        if (has_imag) {
            // COMPLEX-STEP: Use matrix form (I + quatR(delta_quat)) * q
            // quatR(dq) = [sca, -vec^T; vec, sca*I - skew(vec)]
            // (I + quatR(dq)) * q = [1+sca, -vec^T; vec, (1+sca)*I - skew(vec)] * q
            T sca = delta_quat[0];
            Eigen::Matrix<T, 3, 1> vec = delta_quat.template tail<3>();

            // DEBUG: Print intermediate values for first call
            static bool debug_first = true;
            if (debug_first && !std::is_arithmetic<T>::value) {
                std::cout << "DEBUG complex quaternion multiplication:\n";
                std::cout << "  sca = " << sca << "\n";
                std::cout << "  vec = " << vec.transpose() << "\n";
                std::cout << "  quat_vec = " << quat_vec.transpose() << "\n";
                debug_first = false;
            }

            // Scalar part: (1+sca)*q[0] - vec^T*q_vec
            // CRITICAL: Use transpose(), NOT dot(), to avoid complex conjugation in Eigen's dot product
            // Eigen's dot(a,b) computes conj(a)^T * b, but we need a^T * b for complex-step
            auto q_vec_tail = quat_vec.template tail<3>();
            T vec_dot_q = (vec.transpose() * q_vec_tail)(0,0);  // Matrix product gives 1x1 matrix
            quat_new[0] = (T(1.0) + sca) * quat_vec[0] - vec_dot_q;

            // Vector part: vec*q[0] + (1+sca)*q_vec - skew(vec)*q_vec
            //            = vec*q[0] + (1+sca)*q_vec - vec × q_vec
            // CRITICAL: Eigen's cross(a,b) for complex vectors computes conj(a) × b
            // We need -vec × q_vec = q_vec × vec. Compute manually to avoid conjugation.
            // cross(a, b) = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]
            Eigen::Matrix<T, 3, 1> cross_vec_q;
            auto q_vec_3 = quat_vec.template tail<3>();
            // Compute q_vec × vec (not vec × q_vec) to get -vec × q_vec
            cross_vec_q[0] = q_vec_3[1] * vec[2] - q_vec_3[2] * vec[1];
            cross_vec_q[1] = q_vec_3[2] * vec[0] - q_vec_3[0] * vec[2];
            cross_vec_q[2] = q_vec_3[0] * vec[1] - q_vec_3[1] * vec[0];

            // DEBUG
            static bool debug_cross = true;
            if (debug_cross && !std::is_arithmetic<T>::value) {
                std::cout << "  Manual cross_vec_q = " << cross_vec_q.transpose() << "\n";
                std::cout << "  Expected: [0, -ih/2*q[3], +ih/2*q[2]] = [0, -ih/2*0.346, ih/2*0.00274]\n";
                debug_cross = false;
            }

            quat_new.template tail<3>() = vec * quat_vec[0] +
                                          (T(1.0) + sca) * quat_vec.template tail<3>() +
                                          cross_vec_q;

            // DEBUG: Print result for first call
            if (!std::is_arithmetic<T>::value) {
                static bool debug_result = true;
                if (debug_result) {
                    std::cout << "  quat_new = " << quat_new.transpose() << "\n";
                    std::cout << "  imag(quat_new[0]) = " << std::imag(quat_new[0]) << "\n";
                    debug_result = false;
                }
            }
        } else {
            // REAL: Use standard quaternion multiplication
            quat_new[0] = quat_vec[0] * delta_quat[0] - quat_vec.template tail<3>().dot(delta_quat.template tail<3>());
            quat_new.template tail<3>() = quat_vec[0] * delta_quat.template tail<3>() +
                                           delta_quat[0] * quat_vec.template tail<3>() +
                                           quat_vec.template tail<3>().cross(delta_quat.template tail<3>());

            // Normalize quaternion for real types only
            normalizeQuaternionIfReal(quat_new);
        }

        // Update position: transform body-frame linear velocity to world frame
        // p_new = p + R^T * v_body where R = world-to-body rotation matrix
        // CRITICAL: MATLAB's jcalc('Fb', q) calls rq(q(1:4)) which normalizes the quaternion!
        // We must match this exactly: normalize quat_vec before computing rotation matrix
        Eigen::Matrix<T, 4, 1> quat_normalized = quat_vec / quat_vec.norm();

        // Quaternion to rotation matrix (world-to-body)
        T qw = quat_normalized[0], qx = quat_normalized[1], qy = quat_normalized[2], qz = quat_normalized[3];
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

        // Assemble new configuration [pos(3), quat(4)]
        q_new.head(3) = p_new;
        q_new.segment(3, 4) = quat_new;

        // DEBUG: Print for first complex perturbation
        if constexpr (!std::is_arithmetic<T>::value) {
            static bool debug_output = true;
            if (debug_output) {
                std::cout << "[lieGroupConfigurationAddition DEBUG]\n";
                std::cout << "  Input q0 config: pos=" << p.transpose() << ", quat=" << quat_vec.transpose() << "\n";
                std::cout << "  Input dq velocity: omega=" << omega_body.transpose() << ", v=" << v_body.transpose() << "\n";
                std::cout << "  Rotation matrix R (from original quat):\n" << R << "\n";
                std::cout << "  R.transpose() * v_body = " << (R.transpose() * v_body).transpose() << "\n";
                std::cout << "  Output q_new: pos=" << p_new.transpose() << ", quat=" << quat_new.transpose() << "\n";
                debug_output = false;
            }
        }

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
    }
    for (int i = 0; i < qd.size(); ++i) {
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

            // Extract configuration with [pos(3), quat(4)] ordering
            Vec3<double> p = q0.head(3);  // Position in world frame
            Quat<double> quat;
            if (quat_order == 0) {
                quat = q0.segment(3, 4);  // [w,x,y,z]
            } else {
                quat[0] = q0[6];  // w
                quat[1] = q0[3];  // x
                quat[2] = q0[4];  // y
                quat[3] = q0[5];  // z
            }

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

            // Assemble configuration with [pos(3), quat(4)] ordering
            q_new.head(3) = p_new;
            if (quat_order == 0) {
                q_new.segment(3, 4) = quat_new;
            } else {
                q_new[3] = quat_new[1];  // x
                q_new[4] = quat_new[2];  // y
                q_new[5] = quat_new[3];  // z
                q_new[6] = quat_new[0];  // w
            }

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

// Helper function for complex-step differentiation with floating base robots
void testInverseDynamicsDerivativesComplexStepFloatingBase(ClusterTreeModel<double>& model_real,
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
    DVec<double> q0 = state_real.first;  // Make a copy so we can modify it
    const DVec<double>& qd0 = state_real.second;

    // Ensure quaternion is normalized (should already be, but make sure)
    // Configuration ordering is [pos(3), quat(4)], so quaternion is at indices 3-6
    q0.segment<4>(3).normalize();

    // Create complex model (same structure as real model)
    ClusterTreeModel<std::complex<double>> model_complex;

    // Build complex model from the real model
    using namespace ClusterJoints;

    // Iterate over clusters and create each cluster inline (register bodies then create cluster)
    for (size_t cluster_idx = 0; cluster_idx < model_real.clusters().size(); ++cluster_idx) {
        auto cluster = model_real.cluster(cluster_idx);
        const auto& bodies_in_cluster = cluster->bodies();

        // Check if this is a free joint (floating base)
        if (cluster->num_velocities_ == 6 && cluster->num_positions_ == 7) {
            // Free joint (floating base with quaternion) - 1 body
            const auto& body_real = bodies_in_cluster[0];

            SpatialInertia<std::complex<double>> inertia_c(
                std::complex<double>(body_real.inertia_.getMass(), 0.0),
                body_real.inertia_.getCOM().template cast<std::complex<double>>(),
                body_real.inertia_.getInertiaTensor().template cast<std::complex<double>>()
            );

            spatial::Transform<std::complex<double>> Xtree_c(
                body_real.Xtree_.getRotation().template cast<std::complex<double>>(),
                body_real.Xtree_.getTranslation().template cast<std::complex<double>>()
            );

            std::string parent_name = "ground";
            if (body_real.parent_index_ >= 0 && body_real.parent_index_ < model_real.bodies().size()) {
                parent_name = model_real.bodies()[body_real.parent_index_].name_;
            }

            Body<std::complex<double>> body_c = model_complex.registerBody(
                body_real.name_, inertia_c, parent_name, Xtree_c
            );

            model_complex.appendRegisteredBodiesAsCluster<Free<std::complex<double>, ori_representation::Quaternion>>(
                body_real.name_, body_c, body_real.name_ + "_joint"
            );

        } else if (cluster->joint_->type() == ClusterJointTypes::RevoluteWithRotor) {
            // RevoluteWithRotor joint: 2 bodies (link and rotor)
            if (bodies_in_cluster.size() != 2) {
                throw std::runtime_error("RevoluteWithRotor cluster should have exactly 2 bodies");
            }

            const auto& link_body_real = bodies_in_cluster[0];
            const auto& rotor_body_real = bodies_in_cluster[1];

            // Convert link body to complex
            SpatialInertia<std::complex<double>> link_inertia_c(
                std::complex<double>(link_body_real.inertia_.getMass(), 0.0),
                link_body_real.inertia_.getCOM().template cast<std::complex<double>>(),
                link_body_real.inertia_.getInertiaTensor().template cast<std::complex<double>>()
            );

            spatial::Transform<std::complex<double>> link_Xtree_c(
                link_body_real.Xtree_.getRotation().template cast<std::complex<double>>(),
                link_body_real.Xtree_.getTranslation().template cast<std::complex<double>>()
            );

            std::string link_parent_name = "ground";
            if (link_body_real.parent_index_ >= 0 && link_body_real.parent_index_ < model_real.bodies().size()) {
                link_parent_name = model_real.bodies()[link_body_real.parent_index_].name_;
            }

            Body<std::complex<double>> link_body_c = model_complex.registerBody(
                link_body_real.name_, link_inertia_c, link_parent_name, link_Xtree_c
            );

            // Convert rotor body to complex
            SpatialInertia<std::complex<double>> rotor_inertia_c(
                std::complex<double>(rotor_body_real.inertia_.getMass(), 0.0),
                rotor_body_real.inertia_.getCOM().template cast<std::complex<double>>(),
                rotor_body_real.inertia_.getInertiaTensor().template cast<std::complex<double>>()
            );

            spatial::Transform<std::complex<double>> rotor_Xtree_c(
                rotor_body_real.Xtree_.getRotation().template cast<std::complex<double>>(),
                rotor_body_real.Xtree_.getTranslation().template cast<std::complex<double>>()
            );

            std::string rotor_parent_name = "ground";
            if (rotor_body_real.parent_index_ >= 0 && rotor_body_real.parent_index_ < model_real.bodies().size()) {
                rotor_parent_name = model_real.bodies()[rotor_body_real.parent_index_].name_;
            }

            Body<std::complex<double>> rotor_body_c = model_complex.registerBody(
                rotor_body_real.name_, rotor_inertia_c, rotor_parent_name, rotor_Xtree_c
            );

            // Extract gear ratio from loop constraint: G = [1; gear_ratio]
            const DMat<double>& G = cluster->joint_->G();
            double gear_ratio = G(1, 0);

            // Extract axes from motion subspace
            const DMat<double>& S_cluster = cluster->S();

            // Link joint axis (first 6 rows, angular component in rows 0-2)
            ori::CoordinateAxis link_axis;
            if (std::abs(S_cluster(0, 0)) > 0.9) {
                link_axis = ori::CoordinateAxis::X;
            } else if (std::abs(S_cluster(1, 0)) > 0.9) {
                link_axis = ori::CoordinateAxis::Y;
            } else if (std::abs(S_cluster(2, 0)) > 0.9) {
                link_axis = ori::CoordinateAxis::Z;
            } else {
                throw std::runtime_error("Complex-step test only supports axis-aligned revolute joints");
            }

            // Rotor joint axis (next 6 rows, angular component in rows 6-8)
            ori::CoordinateAxis rotor_axis;
            if (std::abs(S_cluster(6, 0)) > 0.9) {
                rotor_axis = ori::CoordinateAxis::X;
            } else if (std::abs(S_cluster(7, 0)) > 0.9) {
                rotor_axis = ori::CoordinateAxis::Y;
            } else if (std::abs(S_cluster(8, 0)) > 0.9) {
                rotor_axis = ori::CoordinateAxis::Z;
            } else {
                throw std::runtime_error("Complex-step test only supports axis-aligned revolute joints");
            }

            // Create geared transmission module
            GearedTransmissionModule<std::complex<double>> module{
                link_body_c,
                rotor_body_c,
                link_body_real.name_ + "_joint",
                rotor_body_real.name_ + "_joint",
                link_axis,
                rotor_axis,
                std::complex<double>(gear_ratio, 0.0)
            };

            model_complex.appendRegisteredBodiesAsCluster<RevoluteWithRotor<std::complex<double>>>(
                link_body_real.name_, module
            );

        } else if (cluster->num_velocities_ == 1 && cluster->num_positions_ == 1) {
            // Simple Revolute joint: 1 body
            const auto& body_real = bodies_in_cluster[0];

            SpatialInertia<std::complex<double>> inertia_c(
                std::complex<double>(body_real.inertia_.getMass(), 0.0),
                body_real.inertia_.getCOM().template cast<std::complex<double>>(),
                body_real.inertia_.getInertiaTensor().template cast<std::complex<double>>()
            );

            spatial::Transform<std::complex<double>> Xtree_c(
                body_real.Xtree_.getRotation().template cast<std::complex<double>>(),
                body_real.Xtree_.getTranslation().template cast<std::complex<double>>()
            );

            std::string parent_name = "ground";
            if (body_real.parent_index_ >= 0 && body_real.parent_index_ < model_real.bodies().size()) {
                parent_name = model_real.bodies()[body_real.parent_index_].name_;
            }

            Body<std::complex<double>> body_c = model_complex.registerBody(
                body_real.name_, inertia_c, parent_name, Xtree_c
            );

            const DMat<double>& S = cluster->S();
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
                body_real.name_, body_c, axis, body_real.name_ + "_joint"
            );

        } else {
            throw std::runtime_error("Complex-step test only supports Free, Revolute, and RevoluteWithRotor joints");
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

    // Test dtau/dq using complex-step with Lie group retraction
    // NOTE: The analytical derivatives dtau/dq are with respect to VELOCITY SPACE perturbations.
    //       We perturb in velocity space and use Lie group retraction to map to configuration space.
    double max_error_dq = 0.0;

    for (int i = 0; i < nDOF; ++i) {
        // Create perturbation in velocity/tangent space
        DVec<std::complex<double>> dq_complex = DVec<std::complex<double>>::Zero(nDOF);
        dq_complex[i] = ih;

        // Apply Lie group retraction: q_perturbed = q0 ⊞ dq
        auto [q_complex, qd_complex] = toComplexState(q0, qd0);

        // DEBUG: Print sizes and first perturbation
        if (i == 0) {
            std::cout << "\nDEBUG first iteration (i=0):\n";
            std::cout << "  q_complex.size() = " << q_complex.size() << "\n";
            std::cout << "  dq_complex.size() = " << dq_complex.size() << "\n";
            std::cout << "  dq_complex[0] = " << dq_complex[0] << "\n";
            std::cout << "  q0.head(7) (FB config) = " << q0.head(7).transpose() << "\n";
        }

        DVec<std::complex<double>> q_perturbed = lieGroupConfigurationAddition(q_complex, dq_complex, true);

        if (i == 0) {
            std::cout << "  q_perturbed.size() = " << q_perturbed.size() << "\n";
            std::cout << "  real(q_perturbed.head(7)) = " << q_perturbed.head(7).real().transpose() << "\n";
            std::cout << "  imag(q_perturbed.head(7)) = " << q_perturbed.head(7).imag().transpose() << "\n\n";
        }

        // Convert to ModelState
        ModelState<std::complex<double>> model_state_complex;
        int idx_q = 0;  // Index into configuration space (size = n_q)
        int idx_v = 0;  // Index into velocity space (size = n_v)
        for (const auto &cluster : model_complex.clusters()) {
            JointCoordinate<std::complex<double>> pos(
                DVec<std::complex<double>>::Zero(cluster->num_positions_), false);
            JointCoordinate<std::complex<double>> vel(
                DVec<std::complex<double>>::Zero(cluster->num_velocities_), false);

            for (int j = 0; j < cluster->num_positions_; ++j) {
                pos[j] = q_perturbed[idx_q++];
            }
            for (int j = 0; j < cluster->num_velocities_; ++j) {
                vel[j] = qd_complex[idx_v++];
            }

            JointState<std::complex<double>> joint_state(pos, vel);
            model_state_complex.push_back(joint_state);
        }

        model_complex.setState(model_state_complex);
        DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

        // Extract derivative from imaginary part
        // tau is in velocity space (nDOF), derivatives are with respect to velocity space coord i
        DVec<double> dtau_dqi_cs(nDOF);
        for (int j = 0; j < nDOF; ++j) {
            dtau_dqi_cs[j] = tau_complex[j].imag() / h;
        }

        // DEBUG: Print first derivatives AND compare to finite differences
        if (i == 0) {
            // Compute finite difference for comparison
            double h_fd = 1e-6;
            DVec<double> dq_fd = DVec<double>::Zero(nDOF);
            dq_fd(0) = h_fd;

            DVec<double> q_fd = lieGroupConfigurationAddition(q0, dq_fd, true);
            ClusterTreeModel<double>::StatePair state_fd = {q_fd, qd0};
            model_real.setState(state_fd);
            DVec<double> tau_fd = model_real.inverseDynamics(ydd_real);
            ClusterTreeModel<double>::StatePair state0_pair = {q0, qd0};
            model_real.setState(state0_pair);
            DVec<double> tau0_real = model_real.inverseDynamics(ydd_real);
            DVec<double> dtau_dq0_fd = (tau_fd - tau0_real) / h_fd;

            std::cout << "  dtau_dq0 (complex-step, first 6): " << dtau_dqi_cs.head(6).transpose() << "\n";
            std::cout << "  dtau_dq0 (finite-diff,  first 6): " << dtau_dq0_fd.head(6).transpose() << "\n";
            std::cout << "  dtau_dq.col(0) (analytical, first 6): " << dtau_dq.col(0).head(6).transpose() << "\n";
            std::cout << "  Difference (CS vs FD): " << (dtau_dqi_cs - dtau_dq0_fd).head(6).transpose() << "\n";
            std::cout << "  Difference (CS vs Analytical): " << (dtau_dqi_cs - dtau_dq.col(0)).head(6).transpose() << "\n";
            std::cout << "  Error (CS vs FD): " << (dtau_dqi_cs - dtau_dq0_fd).norm() << "\n\n";
        }

        // dtau_dq has shape (nDOF, nDOF) - both rows and columns are velocity space
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
        int idx_q = 0;  // Index into configuration space (size = n_q)
        int idx_v = 0;  // Index into velocity space (size = n_v)
        for (const auto &cluster : model_complex.clusters()) {
            JointCoordinate<std::complex<double>> pos(
                DVec<std::complex<double>>::Zero(cluster->num_positions_), false);
            JointCoordinate<std::complex<double>> vel(
                DVec<std::complex<double>>::Zero(cluster->num_velocities_), false);

            for (int j = 0; j < cluster->num_positions_; ++j) {
                pos[j] = q_complex[idx_q++];
            }
            for (int j = 0; j < cluster->num_velocities_; ++j) {
                vel[j] = qd_complex[idx_v++];
            }

            JointState<std::complex<double>> joint_state(pos, vel);
            model_state_complex.push_back(joint_state);
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

// NOTE: Complex-step differentiation CAN work with Lie group manifolds like quaternions!
//
//       The KEY INSIGHT (from MATLAB Spatial_v2 implementation):
//       - For REAL perturbations: Use full exponential map exp(ω)
//       - For COMPLEX perturbations: Use FIRST-ORDER approximation (1 + ω/2)
//
//       The first-order approximation preserves the complex-step derivative property
//       Im(f(x+ih))/h = f'(x) because it's a linear (analytic) function.
//
//       The full exponential map exp(ω) = [cos(||ω||/2), sin(||ω||/2)*ω/||ω||]
//       involves sqrt, sin, cos with complex arguments which breaks analyticity.
//
//       By using the linearized exponential for complex perturbations, we get
//       machine-precision derivatives while maintaining geometric correctness!

TEST(InverseDynamicsDerivativesComplexStep, SimpleFloatingBaseWithRotor) {
    // Create a very simple floating base + 1 revolute with rotor joint model
    using namespace ClusterJoints;
    ClusterTreeModel<double> model;

    // Create floating base body
    SpatialInertia<double> fb_inertia(1.0, Vec3<double>(0, 0, 0), Mat3<double>::Identity() * 0.01);
    Body<double> fb_body = model.registerBody("floating_base", fb_inertia, "ground", spatial::Transform<double>());
    model.appendRegisteredBodiesAsCluster<Free<double, ori_representation::Quaternion>>(
        "floating_base", fb_body, "fb_joint");

    // Create one revolute joint WITH ROTOR attached to floating base
    SpatialInertia<double> link_inertia(0.5, Vec3<double>(0.1, 0, 0), Mat3<double>::Identity() * 0.005);
    SpatialInertia<double> rotor_inertia(0.05, Vec3<double>(0, 0, 0), Mat3<double>::Identity() * 0.0001);
    spatial::Transform<double> Xtree_link(Mat3<double>::Identity(), Vec3<double>(0, 0, 0.5));
    spatial::Transform<double> Xtree_rotor(Mat3<double>::Identity(), Vec3<double>(0, 0, 0.5));

    Body<double> link_body = model.registerBody("link1", link_inertia, "floating_base", Xtree_link);
    Body<double> rotor_body = model.registerBody("rotor1", rotor_inertia, "floating_base", Xtree_rotor);

    GearedTransmissionModule<double> module{link_body, rotor_body,
                                            "link1_joint", "rotor1_joint",
                                            ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                                            6.0};  // gear ratio
    model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<double>>("joint1", module);

    testInverseDynamicsDerivativesComplexStepFloatingBase(model, "Simple Floating Base + 1 Revolute With Rotor", 7);
}

TEST(InverseDynamicsDerivativesComplexStep, SimpleFloatingBase) {
    // Create a very simple floating base + 1 revolute joint model
    using namespace ClusterJoints;
    ClusterTreeModel<double> model;

    // Create floating base body
    SpatialInertia<double> fb_inertia(1.0, Vec3<double>(0, 0, 0), Mat3<double>::Identity() * 0.01);
    Body<double> fb_body = model.registerBody("floating_base", fb_inertia, "ground", spatial::Transform<double>());
    model.appendRegisteredBodiesAsCluster<Free<double, ori_representation::Quaternion>>(
        "floating_base", fb_body, "fb_joint");

    // Create one revolute joint attached to floating base
    SpatialInertia<double> link_inertia(0.5, Vec3<double>(0.1, 0, 0), Mat3<double>::Identity() * 0.005);
    spatial::Transform<double> Xtree(Mat3<double>::Identity(), Vec3<double>(0, 0, 0.5));
    Body<double> link_body = model.registerBody("link1", link_inertia, "floating_base", Xtree);
    model.appendRegisteredBodiesAsCluster<Revolute<double>>(
        "link1", link_body, ori::CoordinateAxis::Z, "link1_joint");

    testInverseDynamicsDerivativesComplexStepFloatingBase(model, "Simple Floating Base + 1 Revolute", 7);
}

TEST(InverseDynamicsDerivativesComplexStep, MiniCheetahQuaternion) {
    MiniCheetah<double, ori_representation::Quaternion> robot;
    ClusterTreeModel<double> model = robot.buildClusterTreeModel();
    testInverseDynamicsDerivativesComplexStepFloatingBase(model, "MiniCheetah (Quaternion)", 18);
}
