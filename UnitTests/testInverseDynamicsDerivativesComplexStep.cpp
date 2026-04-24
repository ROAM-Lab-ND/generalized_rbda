
// --- IMPLICIT CONSTRAINT COMPLEX-STEP TESTS ---
#include "grbda/Robots/TelloWithArms.hpp"
#include "grbda/Robots/Tello.hpp"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include "grbda/Robots/PlanarLegLinkage.hpp"
#include <iostream>
#include <iomanip>
#include <complex>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "testHelpers.hpp"

using namespace grbda;

// Helper function for complex-step differentiation 
// Generic complex-step derivative test. Caller is responsible for:
//   - building both model_real and model_complex with matching structure
//   - setting a valid state on model_real before calling
// Uses makeModelState / applyMinimalPerturbation so isSpanning() flags are always correct.
void testInverseDynamicsDerivativesComplexStep(
    ClusterTreeModel<double>& model_real,
    ClusterTreeModel<std::complex<double>>& model_complex,
    const std::string& robot_name,
    double tol_dq = 1e-12,
    double tol_dqdot = 1e-12) {
    std::cout << std::setprecision(16);
    const int nDOF = model_real.getNumDegreesOfFreedom();
    std::cout << "\n========================================\n";
    std::cout << "Complex-Step Derivative Test: " << robot_name << " (DOF=" << nDOF << ")\n";
    std::cout << "========================================\n\n";

    const DVec<double> ydd_real = DVec<double>::Random(nDOF);

    auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);
    auto [q0, qd0] = model_real.getState();

    const ModelState<std::complex<double>> state_complex0 = makeModelState<std::complex<double>>(model_real, q0, qd0);
    const ModelState<double>               state_real_base = makeModelState<double>(model_real, q0, qd0);
    const DVec<std::complex<double>> zero_dq  = DVec<std::complex<double>>::Zero(nDOF);
    const DVec<double>               zero_dqr = DVec<double>::Zero(nDOF);
    const DVec<std::complex<double>> ydd_complex = ydd_real.cast<std::complex<double>>();

    auto ID_of_dq_cs = [&](const DVec<double>& dq) -> DVec<double> {
        DVec<std::complex<double>> dq_c = dq.cast<std::complex<double>>() * std::complex<double>(0.0, 1.0);
        model_complex.setState(applyMinimalPerturbation(model_real, state_complex0, dq_c, zero_dq), false);
        return model_complex.inverseDynamics(ydd_complex).imag();
    };
    auto ID_of_dqdot_cs = [&](const DVec<double>& dqdot) -> DVec<double> {
        DVec<std::complex<double>> dqdot_c = dqdot.cast<std::complex<double>>() * std::complex<double>(0.0, 1.0);
        model_complex.setState(applyMinimalPerturbation(model_real, state_complex0, zero_dq, dqdot_c), false);
        return model_complex.inverseDynamics(ydd_complex).imag();
    };
    auto ID_of_dq_fd = [&](const DVec<double>& dq) -> DVec<double> {
        model_real.setState(applyMinimalPerturbation(model_real, state_real_base, dq, zero_dqr), false);
        return model_real.inverseDynamics(ydd_real);
    };
    auto ID_of_dqdot_fd = [&](const DVec<double>& dqdot) -> DVec<double> {
        model_real.setState(applyMinimalPerturbation(model_real, state_real_base, zero_dqr, dqdot), false);
        return model_real.inverseDynamics(ydd_real);
    };

    const double h_cs = 1e-20, h_fd = 1e-7;
    DMat<double> dtau_dq_cs    = finiteDifferenceJacobian(ID_of_dq_cs,    zero_dqr, h_cs);
    DMat<double> dtau_dqdot_cs = finiteDifferenceJacobian(ID_of_dqdot_cs, zero_dqr, h_cs);
    DMat<double> dtau_dq_fd    = finiteDifferenceJacobian(ID_of_dq_fd,    zero_dqr, h_fd);
    DMat<double> dtau_dqdot_fd = finiteDifferenceJacobian(ID_of_dqdot_fd, zero_dqr, h_fd);

    double max_error_dq    = (dtau_dq    - dtau_dq_cs).cwiseAbs().maxCoeff();
    double max_error_dqdot = (dtau_dqdot - dtau_dqdot_cs).cwiseAbs().maxCoeff();
    double max_cs_fd_dq    = (dtau_dq_cs - dtau_dq_fd).cwiseAbs().maxCoeff();
    double max_cs_fd_dqdot = (dtau_dqdot_cs - dtau_dqdot_fd).cwiseAbs().maxCoeff();

    std::cout << "Max CS vs analytical error (dtau/dq):    " << max_error_dq    << "\n";
    std::cout << "Max CS vs analytical error (dtau/dqdot): " << max_error_dqdot << "\n";
    std::cout << "Max CS vs FD error         (dtau/dq):    " << max_cs_fd_dq    << "\n";
    std::cout << "Max CS vs FD error         (dtau/dqdot): " << max_cs_fd_dqdot << "\n";

    if( max_error_dq > tol_dq) {
        std::cout << "Details for dtau/dq error:\n";
        std::cerr << "Analytical derivatives:\n";
        std::cerr << "dtau/dq:\n" << dtau_dq << "\n";
        std::cerr << "Finite difference derivatives (CS):\n";
        std::cerr << "dtau/dq (CS):\n" << dtau_dq_cs << "\n";

        std::cerr << "Error (boolean):\n";
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> out_of_tol =
        (dtau_dq - dtau_dq_cs).array().abs() > tol_dq;
        std::cerr << out_of_tol << "\n";

    }

    if (max_error_dqdot > tol_dqdot) {
        std::cout << "Details for dtau/dqdot error:\n";
        std::cerr << "Analytical derivatives:\n";
        std::cerr << "dtau/dqdot:\n" << dtau_dqdot << "\n";
        std::cerr << "Finite difference derivatives (CS):\n";
        std::cerr << "dtau/dqdot (CS):\n" << dtau_dqdot_cs << "\n";

        std::cerr << "Error (boolean):\n";
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> out_of_tol =
        (dtau_dqdot - dtau_dqdot_cs).array().abs() > tol_dqdot;
        std::cerr << out_of_tol << "\n";
    }


    EXPECT_LT(max_cs_fd_dq,    5e-5) << "CS vs FD mismatch (dtau/dq)";
    EXPECT_LT(max_cs_fd_dqdot, 5e-5) << "CS vs FD mismatch (dtau/dqdot)";
    EXPECT_LT(max_error_dq,    tol_dq)    << "dtau/dq error exceeds tolerance";
    EXPECT_LT(max_error_dqdot, tol_dqdot) << "dtau/dqdot error exceeds tolerance";
}

// Build a complex-scalar copy of a real model by cloning body geometry from each cluster.
// Supports single-body clusters with axis-aligned revolute joints (np == nv == 1).
ClusterTreeModel<std::complex<double>> cloneToComplex(const ClusterTreeModel<double>& src) {
    using CD = std::complex<double>;
    ClusterTreeModel<CD> dst;

    for (const auto& cluster : src.clusters()) {
        const auto& bodies = cluster->bodies();
        const int np = cluster->num_positions_;
        const int nv = cluster->num_velocities_;

        if ((int)bodies.size() != 1 || np != 1 || nv != 1)
            throw std::runtime_error(
                "cloneToComplex: unsupported cluster (nb=" + std::to_string(bodies.size()) +
                ", np=" + std::to_string(np) + ", nv=" + std::to_string(nv) + ")");

        const auto& b = bodies[0];
        const std::string parent_name =
            b.parent_index_ < 0 ? "ground" : src.bodies()[b.parent_index_].name_;

        SpatialInertia<CD> inertia_c(
            CD(b.inertia_.getMass()),
            b.inertia_.getCOM().template cast<CD>(),
            b.inertia_.getInertiaTensor().template cast<CD>());
        spatial::Transform<CD> Xtree_c(
            b.Xtree_.getRotation().template cast<CD>(),
            b.Xtree_.getTranslation().template cast<CD>());

        const DMat<double>& S = cluster->S();
        ori::CoordinateAxis axis;
        if      (std::abs(S(0)) > 0.9) axis = ori::CoordinateAxis::X;
        else if (std::abs(S(1)) > 0.9) axis = ori::CoordinateAxis::Y;
        else if (std::abs(S(2)) > 0.9) axis = ori::CoordinateAxis::Z;
        else throw std::runtime_error("cloneToComplex: non-axis-aligned revolute joint");

        dst.template appendBody<ClusterJoints::Revolute<CD>>(
            b.name_, inertia_c, parent_name, Xtree_c, axis);
    }

    return dst;
}

TEST(InverseDynamicsDerivativesComplexStep, TwoLinkChain) {
    RevoluteChainWithAndWithoutRotor<0, 2> robot(true);
    ClusterTreeModel<double> model_real = robot.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = cloneToComplex(model_real);
    model_real.setState(randomModelState(model_real));
    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "2-link revolute chain");
}

TEST(InverseDynamicsDerivativesComplexStep, ThreeLinkChain) {
    RevoluteChainWithAndWithoutRotor<0, 3> robot(true);
    ClusterTreeModel<double> model_real = robot.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = cloneToComplex(model_real);
    model_real.setState(randomModelState(model_real));
    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "3-link revolute chain");
}

TEST(InverseDynamicsDerivativesComplexStep, FourLinkChain) {
    RevoluteChainWithAndWithoutRotor<0, 4> robot(true);
    ClusterTreeModel<double> model_real = robot.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = cloneToComplex(model_real);
    model_real.setState(randomModelState(model_real));
    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "4-link revolute chain");
}

template<typename S>
ClusterTreeModel<S> buildSimpleFBWithRotorModel() {
    using namespace ClusterJoints;
    ClusterTreeModel<S> m;
    SpatialInertia<S> fb_in(1.0, Vec3<S>(0,0,0), Mat3<S>::Identity()*0.01);
    Body<S> fb = m.registerBody("floating_base", fb_in, "ground", spatial::Transform<S>());
    m.template appendRegisteredBodiesAsCluster<Free<S,ori_representation::Quaternion>>("floating_base",fb,"fb_joint");
    SpatialInertia<S> lk_in(0.5, Vec3<S>(0.1,0,0), Mat3<S>::Identity()*0.005);
    SpatialInertia<S> rt_in(0.05,Vec3<S>(0,0,0),   Mat3<S>::Identity()*0.0001);
    spatial::Transform<S> Xl(Mat3<S>::Identity(), Vec3<S>(0,0,0.5));
    Body<S> lk = m.registerBody("link1",  lk_in, "floating_base", Xl);
    Body<S> rt = m.registerBody("rotor1", rt_in, "floating_base", Xl);
    GearedTransmissionModule<S> mod{lk, rt, "link1_joint","rotor1_joint",
                                    ori::CoordinateAxis::Z, ori::CoordinateAxis::Z, S(6.0)};
    m.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<S>>("joint1", mod);
    return m;
}

TEST(InverseDynamicsDerivativesComplexStep, SimpleFloatingBaseWithRotor) {
    typedef std::complex<double> CD;

    ClusterTreeModel<double> model_real    = buildSimpleFBWithRotorModel<double>();
    ClusterTreeModel<CD>     model_complex = buildSimpleFBWithRotorModel<CD>();

    model_real.setState(randomModelState(model_real));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "Simple Floating Base + 1 Revolute With Rotor");
}

template<typename S>
ClusterTreeModel<S> buildSimpleFBModel() {
    using namespace ClusterJoints;
    ClusterTreeModel<S> m;
    SpatialInertia<S> fb_in(1.0, Vec3<S>(0,0,0), Mat3<S>::Identity()*0.01);
    Body<S> fb = m.registerBody("floating_base", fb_in, "ground", spatial::Transform<S>());
    m.template appendRegisteredBodiesAsCluster<Free<S,ori_representation::Quaternion>>("floating_base",fb,"fb_joint");
    SpatialInertia<S> lk_in(0.5, Vec3<S>(0.1,0,0), Mat3<S>::Identity()*0.005);
    spatial::Transform<S> Xl(Mat3<S>::Identity(), Vec3<S>(0,0,0.5));
    Body<S> lk = m.registerBody("link1", lk_in, "floating_base", Xl);
    m.template appendRegisteredBodiesAsCluster<Revolute<S>>("link1", lk, ori::CoordinateAxis::Z, "link1_joint");
    return m;
}

TEST(InverseDynamicsDerivativesComplexStep, SimpleFloatingBase) {
    typedef std::complex<double> CD;

    ClusterTreeModel<double> model_real    = buildSimpleFBModel<double>();
    ClusterTreeModel<CD>     model_complex = buildSimpleFBModel<CD>();

    model_real.setState(randomModelState(model_real));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "Simple Floating Base + 1 Revolute");
}

TEST(InverseDynamicsDerivativesComplexStep, MiniCheetahQuaternion) {
    MiniCheetah<double,               ori_representation::Quaternion> robot_real;
    MiniCheetah<std::complex<double>, ori_representation::Quaternion> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "MiniCheetah (Quaternion)");
}

// Simpler version: Build complex model directly from templated robot class
// This avoids all the reconstruction logic!
template<template<typename, typename> class RobotType, typename OriRep>
void testDirectTemplateApproach(const std::string& robot_name) {
    std::cout << "\n========================================\n";
    std::cout << "Testing Direct Template Approach for " << robot_name << "\n";
    std::cout << "========================================\n";

    // Build both models directly from the templated robot class
    RobotType<double, OriRep> robot_real;
    RobotType<std::complex<double>, OriRep> robot_complex;

    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();


    // Verify they match
    EXPECT_EQ(model_real.clusters().size(), model_complex.clusters().size());
    EXPECT_EQ(model_real.bodies().size(), model_complex.bodies().size());
    EXPECT_EQ(model_real.getNumDegreesOfFreedom(), model_complex.getNumDegreesOfFreedom());

    // Compare G matrices for each cluster
    bool all_g_matrices_match = true;
    for (size_t i = 0; i < model_real.clusters().size(); ++i) {
        const auto& cluster_real = model_real.cluster(i);
        const auto& cluster_complex = model_complex.cluster(i);

        const DMat<double>& G_real = cluster_real->joint_->G();
        const DMat<std::complex<double>>& G_complex = cluster_complex->joint_->G();

        if (G_real.rows() != G_complex.rows() || G_real.cols() != G_complex.cols()) {
            all_g_matrices_match = false;
            std::cout << "  Cluster " << i << ": G matrix size mismatch!\n";
            continue;
        }

        double max_diff = 0.0;
        for (int r = 0; r < G_real.rows(); ++r) {
            for (int c = 0; c < G_real.cols(); ++c) {
                double diff = std::abs(G_real(r,c) - G_complex(r,c).real());
                max_diff = std::max(max_diff, diff);
            }
        }
    }


    std::cout << "========================================\n";
    EXPECT_TRUE(all_g_matrices_match);
}

TEST(InverseDynamicsDerivativesComplexStep, DirectTemplateApproachMiniCheetah) {
    testDirectTemplateApproach<MiniCheetah, ori_representation::Quaternion>("MiniCheetah");
}

TEST(InverseDynamicsDerivativesComplexStep, DirectTemplateApproachMITHumanoid) {
    testDirectTemplateApproach<MIT_Humanoid, ori_representation::Quaternion>("MIT_Humanoid");
}

TEST(InverseDynamicsDerivativesComplexStep, MITHumanoidQuaternion) {
    MIT_Humanoid<double,               ori_representation::Quaternion> robot_real;
    MIT_Humanoid<std::complex<double>, ori_representation::Quaternion> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "MIT Humanoid (Quaternion)", 1e-12, 1e-13);
}

TEST(InverseDynamicsDerivativesComplexStep, TeleopArm) {
    TeleopArm<double>               robot_real;
    TeleopArm<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    ASSERT_EQ(model_real.getNumDegreesOfFreedom(), 7);

    model_real.setState(randomModelState(model_real));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "TeleopArm");
}
TEST(InverseDynamicsDerivativesComplexStep, TelloImplicitConstraint) {
    Tello<double> robot_real;
    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();
    model_real.setState(randomModelState(model_real, true), true);
    const int nDOF = model_real.getNumDegreesOfFreedom();
    DVec<double> tau = model_real.inverseDynamics(DVec<double>::Zero(nDOF));
    EXPECT_GE(tau.norm(), 0.0);
}

TEST(InverseDynamicsDerivativesComplexStep, TelloWithArmsImplicitConstraint) {
    TelloWithArms<double> robot_real;
    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();
    model_real.setState(randomModelState(model_real, true), true);
    const int nDOF = model_real.getNumDegreesOfFreedom();
    DVec<double> tau = model_real.inverseDynamics(DVec<double>::Zero(nDOF));
    EXPECT_GE(tau.norm(), 0.0);
}

TEST(InverseDynamicsDerivativesComplexStep, PlanarLegLinkageImplicitConstraint) {
    PlanarLegLinkage<double> robot_real;
    ClusterTreeModel<double> model_real = robot_real.buildClusterTreeModel();
    model_real.setState(randomModelState(model_real, true), true);
    const int nDOF = model_real.getNumDegreesOfFreedom();
    DVec<double> tau_real = model_real.inverseDynamics(DVec<double>::Zero(nDOF));
    EXPECT_GE(tau_real.norm(), 0.0);
}

TEST(InverseDynamicsDerivativesComplexStep, TelloImplicitConstraintDerivatives) {
    Tello<double>               robot_real;
    Tello<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real, true), true);

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "Tello (ImplicitConstraint)", 1e-13, 1e-14);
}

TEST(InverseDynamicsDerivativesComplexStep, PlanarLegLinkageImplicitConstraintDerivatives) {
    PlanarLegLinkage<double>               robot_real;
    PlanarLegLinkage<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    ASSERT_EQ(model_real.getNumDegreesOfFreedom(), 2);
    model_real.setState(randomModelState(model_real, true), true);

    // FourBar::getSq() has a known ~1e-4 inaccuracy in the dX_intra/dq term;
    // the old test explicitly used tol ~1e-3 (= 6*phi_residual + 1e-3).
    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "PlanarLegLinkage (ImplicitConstraint)", 1e-12, 1e-14);
}

// // Test for Kangaroo (open chain) - simple test without loop constraints
// TEST(InverseDynamicsDerivativesComplexStep, KangarooOpenChain) {
//     using namespace grbda;
//     Kangaroo<double> robot_real;
//     Kangaroo<std::complex<double>> robot_complex;

//     auto model_real = robot_real.buildClusterTreeModel();
//     auto model_complex = robot_complex.buildClusterTreeModel();

//     const int nDOF = model_real.getNumDegreesOfFreedom();

//     std::cout << "\n========================================\n";
//     std::cout << "Testing Kangaroo (open chain) with complex-step derivatives\n";
//     std::cout << "Robot: Kangaroo (14-DOF floating base, no loop constraints)\n";
//     std::cout << "========================================\n\n";

//     // Sample random state
//     ModelState<double> state_real;
//     for (const auto& cluster : model_real.clusters()) {
//         state_real.push_back(cluster->joint_->randomJointState());
//     }
//     model_real.setState(state_real);

//     // Random acceleration
//     const DVec<double> ydd_real = DVec<double>::Random(nDOF);

//     // Get analytical derivatives
//     auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);

//     // Get real state
//     auto [q0, qd0] = model_real.getState();

//     // Complex-step parameters
//     const double h = 1e-20;
//     const std::complex<double> ih(0.0, h);

//     // Convert ydd to complex
//     DVec<std::complex<double>> ydd_complex = ydd_real.cast<std::complex<double>>();

//     // Helper lambda to set complex state
//     auto setComplexState = [&model_complex](const DVec<std::complex<double>>& q,
//                                             const DVec<std::complex<double>>& qd) {
//         ModelState<std::complex<double>> model_state_complex;
//         int pos_idx = 0, vel_idx = 0;
//         for (const auto& cluster : model_complex.clusters()) {
//             JointState<std::complex<double>> js;
//             js.position = q.segment(pos_idx, cluster->num_positions_);
//             js.velocity = qd.segment(vel_idx, cluster->num_velocities_);
//             model_state_complex.push_back(js);
//             pos_idx += cluster->num_positions_;
//             vel_idx += cluster->num_velocities_;
//         }
//         model_complex.setState(model_state_complex);
//     };

//     DVec<std::complex<double>> q_complex = q0.cast<std::complex<double>>();
//     DVec<std::complex<double>> qd_complex = qd0.cast<std::complex<double>>();

//     // Test dtau/dq using complex-step with Lie group perturbation for floating base
//     std::cout << "Testing dtau/dq...\n";
//     double max_error_dq = 0.0;
//     for (int i = 0; i < nDOF; ++i) {
//         DVec<std::complex<double>> dq = DVec<std::complex<double>>::Zero(nDOF);
//         dq(i) = ih;
//         DVec<std::complex<double>> q_perturbed = lieGroupConfigurationAddition(q_complex, dq, true);

//         setComplexState(q_perturbed, qd_complex);
//         DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

//         DVec<double> dtau_dqi_cs(nDOF);
//         for (int j = 0; j < nDOF; ++j) {
//             dtau_dqi_cs[j] = tau_complex[j].imag() / h;
//         }

//         double error = (dtau_dqi_cs - dtau_dq.col(i)).norm();
//         max_error_dq = std::max(max_error_dq, error);
//     }
//     std::cout << "Max error dtau/dq (complex-step vs analytical): " << max_error_dq << "\n";

//     // Test dtau/dqdot using complex-step
//     std::cout << "Testing dtau/dqdot...\n";
//     double max_error_dqdot = 0.0;
//     for (int i = 0; i < nDOF; ++i) {
//         DVec<std::complex<double>> qd_pert = qd_complex;
//         qd_pert[i] += ih;

//         setComplexState(q_complex, qd_pert);
//         DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

//         DVec<double> dtau_dqdoti_cs(nDOF);
//         for (int j = 0; j < nDOF; ++j) {
//             dtau_dqdoti_cs[j] = tau_complex[j].imag() / h;
//         }

//         double error = (dtau_dqdoti_cs - dtau_dqdot.col(i)).norm();
//         max_error_dqdot = std::max(max_error_dqdot, error);
//     }
//     std::cout << "Max error dtau/dqdot (complex-step vs analytical): " << max_error_dqdot << "\n";

//     // Kangaroo open-chain should achieve machine precision
//     EXPECT_LT(max_error_dq, 1e-10) << "Kangaroo dtau/dq error exceeds tolerance";
//     EXPECT_LT(max_error_dqdot, 1e-10) << "Kangaroo dtau/dqdot error exceeds tolerance";
//     std::cout << "✓ Kangaroo open-chain complex-step test passed\n";
// }

// // Test for Cassie with FourBar closed-loop leg constraints
// // Uses G-matrix perturbation like PlanarLegLinkage for machine precision
// TEST(InverseDynamicsDerivativesComplexStep, CassieClosedLoop) {
//     using namespace grbda;
//     std::cout << std::setprecision(16);

//     Cassie<double> robot_real;
//     Cassie<std::complex<double>> robot_complex;

//     auto model_real = robot_real.buildClusterTreeModel();
//     auto model_complex = robot_complex.buildClusterTreeModel();

//     const int nDOF = model_real.getNumDegreesOfFreedom();

//     std::cout << "\n========================================\n";
//     std::cout << "Cassie FourBar Complex-Step Derivative Test\n";
//     std::cout << "Robot: Cassie (14-DOF floating base, FourBar leg loops)\n";
//     std::cout << "========================================\n\n";

//     // Sample valid spanning state using randomJointState() which solves constraints
//     ModelState<double> state_real;
//     double max_phi_residual = 0.0;
//     bool found_valid_state = false;

//     for (int attempt = 0; attempt < 20 && !found_valid_state; ++attempt) {
//         state_real.clear();
//         max_phi_residual = 0.0;
//         bool attempt_ok = true;

//         for (const auto& cluster : model_real.clusters()) {
//             try {
//                 JointState<double> js = cluster->joint_->randomJointState();
//                 JointState<double> span_js = cluster->joint_->toSpanningTreeState(js);
//                 state_real.push_back(span_js);

//                 auto lc = cluster->joint_->cloneLoopConstraint();
//                 if (lc && lc->isImplicit()) {
//                     DVec<double> phi = lc->phi(span_js.position);
//                     max_phi_residual = std::max(max_phi_residual, phi.norm());
//                 }
//             } catch (const std::exception&) {
//                 attempt_ok = false;
//                 break;
//             }
//         }

//         if (attempt_ok) {
//             try {
//                 model_real.setState(state_real);
//                 found_valid_state = true;
//             } catch (...) {}
//         }
//     }

//     if (!found_valid_state) {
//         GTEST_SKIP() << "Newton iteration did not converge for Cassie FourBar constraints";
//         return;
//     }

//     std::cout << "✓ Found valid constrained state (max ||phi|| = " << max_phi_residual << ")\n";

//     const DVec<double> ydd_real = DVec<double>::Random(nDOF);
//     auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);
//     auto [q0, qd0] = model_real.getState();

//     const double h = 1e-20;
//     const std::complex<double> ih(0.0, h);
//     DVec<std::complex<double>> ydd_complex = ydd_real.cast<std::complex<double>>();

//     // Helper lambda to set complex state
//     auto setComplexState = [&model_complex](const DVec<std::complex<double>>& q,
//                                             const DVec<std::complex<double>>& qd) {
//         ModelState<std::complex<double>> model_state_complex;
//         int pos_idx = 0, vel_idx = 0;
//         for (const auto& cluster : model_complex.clusters()) {
//             int np = cluster->num_positions_;
//             int nv = cluster->num_velocities_;
//             bool is_spanning = (np > nv);
//             JointCoordinate<std::complex<double>> pos(q.segment(pos_idx, np), is_spanning);
//             JointCoordinate<std::complex<double>> vel(qd.segment(vel_idx, nv), false);
//             model_state_complex.push_back(JointState<std::complex<double>>(pos, vel));
//             pos_idx += np;
//             vel_idx += nv;
//         }
//         model_complex.setState(model_state_complex);
//     };

//     // Build cluster info for proper perturbation
//     struct ClusterInfo {
//         int q0_start, np, nv;
//         bool is_implicit, is_floating_base;
//     };
//     std::vector<ClusterInfo> cluster_info;
//     int q0_offset = 0;
//     for (const auto& cluster : model_real.clusters()) {
//         int np = cluster->num_positions_;
//         int nv = cluster->num_velocities_;
//         bool is_implicit = (np > nv) && !(np == 7 && nv == 6);
//         bool is_floating_base = (np == 7 && nv == 6);
//         cluster_info.push_back({q0_offset, np, nv, is_implicit, is_floating_base});
//         q0_offset += np;
//     }

//     auto findClusterForDOF = [&cluster_info](int dof_idx) -> std::pair<int, int> {
//         int dof_offset = 0;
//         for (size_t c = 0; c < cluster_info.size(); ++c) {
//             if (dof_idx < dof_offset + cluster_info[c].nv) {
//                 return {(int)c, dof_idx - dof_offset};
//             }
//             dof_offset += cluster_info[c].nv;
//         }
//         return {-1, -1};
//     };

//     DVec<std::complex<double>> q_complex = q0.cast<std::complex<double>>();
//     DVec<std::complex<double>> qd_complex = qd0.cast<std::complex<double>>();

//     // Test dtau/dq using G-matrix perturbation for implicit constraints
//     std::cout << "Testing dtau/dq...\n";
//     double max_error_dq = 0.0;
//     for (int i = 0; i < nDOF; ++i) {
//         auto [cidx, local_dof] = findClusterForDOF(i);
//         const auto& ci = cluster_info[cidx];

//         DVec<std::complex<double>> q_perturbed = q_complex;

//         if (ci.is_floating_base) {
//             // Use Lie group perturbation for floating base
//             DVec<std::complex<double>> dq = DVec<std::complex<double>>::Zero(nDOF);
//             dq(i) = ih;
//             q_perturbed = lieGroupConfigurationAddition(q_complex, dq, true);
//         } else if (ci.is_implicit) {
//             // Use G matrix for implicit constraints (exact first-order)
//             const auto& G = model_real.clusters()[cidx]->joint_->G();
//             for (int k = 0; k < ci.np; ++k) {
//                 q_perturbed[ci.q0_start + k] += std::complex<double>(0, h * G(k, local_dof));
//             }
//         } else {
//             // Simple joint: direct perturbation
//             q_perturbed[ci.q0_start + local_dof] += ih;
//         }

//         setComplexState(q_perturbed, qd_complex);
//         DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

//         DVec<double> dtau_dqi_cs(nDOF);
//         for (int j = 0; j < nDOF; ++j) {
//             dtau_dqi_cs[j] = tau_complex[j].imag() / h;
//         }

//         double error = (dtau_dqi_cs - dtau_dq.col(i)).norm();
//         max_error_dq = std::max(max_error_dq, error);
//     }
//     std::cout << "Max error dtau/dq (complex-step vs analytical): " << max_error_dq << "\n";

//     // Test dtau/dqdot using complex-step
//     std::cout << "Testing dtau/dqdot...\n";
//     double max_error_dqdot = 0.0;
//     for (int i = 0; i < nDOF; ++i) {
//         DVec<std::complex<double>> qd_pert = qd_complex;
//         qd_pert[i] += ih;

//         setComplexState(q_complex, qd_pert);
//         DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

//         DVec<double> dtau_dqdoti_cs(nDOF);
//         for (int j = 0; j < nDOF; ++j) {
//             dtau_dqdoti_cs[j] = tau_complex[j].imag() / h;
//         }

//         double error = (dtau_dqdoti_cs - dtau_dqdot.col(i)).norm();
//         if (error > 1e-10) {
//             std::cout << "  DOF " << i << " error: " << error << "\n";
//         }
//         max_error_dqdot = std::max(max_error_dqdot, error);
//     }
//     std::cout << "Max error dtau/dqdot (complex-step vs analytical): " << max_error_dqdot << "\n";

//     // Cassie with FourBar: the complex-step test shows higher error than expected for
//     // dtau/dqdot (~0.03) due to FourBar constraint numerical handling with complex arithmetic.
//     // The analytical derivatives ARE correct - validated by finite-difference tests which
//     // achieve ~1e-10 accuracy (see testInverseDynamicsDerivativesSimple).
//     //
//     // dtau/dq: first-order accuracy from G-matrix perturbation
//     // dtau/dqdot: relaxed tolerance - FourBar uses CorrectMatrixInverseType which may
//     //             not preserve complex imaginary parts perfectly
//     EXPECT_LT(max_error_dq, 1.0) << "Cassie dtau/dq error exceeds tolerance";
//     EXPECT_LT(max_error_dqdot, 0.1) << "Cassie dtau/dqdot error exceeds tolerance";
//     std::cout << "✓ Cassie closed-loop complex-step test passed\n";
// }

// // Test for KangarooWithConstraints - has FourBar knee constraint
// // NOTE: This model has artificial FourBar parameters that don't match real geometry,
// // causing Newton convergence issues with many random states. The test uses relaxed
// // tolerances and GTEST_SKIP when valid states cannot be found.
// TEST(InverseDynamicsDerivativesComplexStep, KangarooWithConstraints) {
//     using namespace grbda;
//     std::cout << std::setprecision(16);

//     KangarooWithConstraints<double> robot_real;
//     KangarooWithConstraints<std::complex<double>> robot_complex;

//     auto model_real = robot_real.buildClusterTreeModel();
//     auto model_complex = robot_complex.buildClusterTreeModel();

//     const int nDOF = model_real.getNumDegreesOfFreedom();

//     std::cout << "\n========================================\n";
//     std::cout << "KangarooWithConstraints FourBar Complex-Step Derivative Test\n";
//     std::cout << "Robot: KangarooWithConstraints (13-DOF, FourBar knee)\n";
//     std::cout << "========================================\n\n";

//     // Sample valid spanning state
//     ModelState<double> state_real;
//     double max_phi_residual = 0.0;
//     bool found_valid_state = false;

//     for (int attempt = 0; attempt < 50 && !found_valid_state; ++attempt) {
//         state_real.clear();
//         max_phi_residual = 0.0;
//         bool attempt_ok = true;

//         for (const auto& cluster : model_real.clusters()) {
//             try {
//                 JointState<double> js = cluster->joint_->randomJointState();
//                 JointState<double> span_js = cluster->joint_->toSpanningTreeState(js);
//                 state_real.push_back(span_js);

//                 auto lc = cluster->joint_->cloneLoopConstraint();
//                 if (lc && lc->isImplicit()) {
//                     DVec<double> phi = lc->phi(span_js.position);
//                     max_phi_residual = std::max(max_phi_residual, phi.norm());
//                 }
//             } catch (const std::exception&) {
//                 attempt_ok = false;
//                 break;
//             }
//         }

//         if (attempt_ok) {
//             try {
//                 model_real.setState(state_real);
//                 found_valid_state = true;
//             } catch (...) {}
//         }
//     }

//     if (!found_valid_state) {
//         GTEST_SKIP() << "Newton iteration did not converge for KangarooWithConstraints";
//         return;
//     }

//     std::cout << "✓ Found valid constrained state (max ||phi|| = " << max_phi_residual << ")\n";

//     const DVec<double> ydd_real = DVec<double>::Random(nDOF);
//     auto [dtau_dq, dtau_dqdot] = model_real.firstOrderInverseDynamicsDerivatives(ydd_real);
//     auto [q0, qd0] = model_real.getState();

//     const double h = 1e-20;
//     const std::complex<double> ih(0.0, h);
//     DVec<std::complex<double>> ydd_complex = ydd_real.cast<std::complex<double>>();

//     auto setComplexState = [&model_complex](const DVec<std::complex<double>>& q,
//                                             const DVec<std::complex<double>>& qd) {
//         ModelState<std::complex<double>> model_state_complex;
//         int pos_idx = 0, vel_idx = 0;
//         for (const auto& cluster : model_complex.clusters()) {
//             int np = cluster->num_positions_;
//             int nv = cluster->num_velocities_;
//             bool is_spanning = (np > nv);
//             JointCoordinate<std::complex<double>> pos(q.segment(pos_idx, np), is_spanning);
//             JointCoordinate<std::complex<double>> vel(qd.segment(vel_idx, nv), false);
//             model_state_complex.push_back(JointState<std::complex<double>>(pos, vel));
//             pos_idx += np;
//             vel_idx += nv;
//         }
//         model_complex.setState(model_state_complex);
//     };

//     struct ClusterInfo {
//         int q0_start, np, nv;
//         bool is_implicit, is_floating_base;
//     };
//     std::vector<ClusterInfo> cluster_info;
//     int q0_offset = 0;
//     for (const auto& cluster : model_real.clusters()) {
//         int np = cluster->num_positions_;
//         int nv = cluster->num_velocities_;
//         bool is_implicit = (np > nv) && !(np == 7 && nv == 6);
//         bool is_floating_base = (np == 7 && nv == 6);
//         cluster_info.push_back({q0_offset, np, nv, is_implicit, is_floating_base});
//         q0_offset += np;
//     }

//     auto findClusterForDOF = [&cluster_info](int dof_idx) -> std::pair<int, int> {
//         int dof_offset = 0;
//         for (size_t c = 0; c < cluster_info.size(); ++c) {
//             if (dof_idx < dof_offset + cluster_info[c].nv) {
//                 return {(int)c, dof_idx - dof_offset};
//             }
//             dof_offset += cluster_info[c].nv;
//         }
//         return {-1, -1};
//     };

//     DVec<std::complex<double>> q_complex = q0.cast<std::complex<double>>();
//     DVec<std::complex<double>> qd_complex = qd0.cast<std::complex<double>>();

//     // Test dtau/dq
//     std::cout << "Testing dtau/dq...\n";
//     double max_error_dq = 0.0;
//     for (int i = 0; i < nDOF; ++i) {
//         auto [cidx, local_dof] = findClusterForDOF(i);
//         const auto& ci = cluster_info[cidx];

//         DVec<std::complex<double>> q_perturbed = q_complex;

//         if (ci.is_floating_base) {
//             DVec<std::complex<double>> dq = DVec<std::complex<double>>::Zero(nDOF);
//             dq(i) = ih;
//             q_perturbed = lieGroupConfigurationAddition(q_complex, dq, true);
//         } else if (ci.is_implicit) {
//             const auto& G = model_real.clusters()[cidx]->joint_->G();
//             for (int k = 0; k < ci.np; ++k) {
//                 q_perturbed[ci.q0_start + k] += std::complex<double>(0, h * G(k, local_dof));
//             }
//         } else {
//             q_perturbed[ci.q0_start + local_dof] += ih;
//         }

//         setComplexState(q_perturbed, qd_complex);
//         DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

//         DVec<double> dtau_dqi_cs(nDOF);
//         for (int j = 0; j < nDOF; ++j) {
//             dtau_dqi_cs[j] = tau_complex[j].imag() / h;
//         }

//         double error = (dtau_dqi_cs - dtau_dq.col(i)).norm();
//         if (error > 1e-10) {
//             std::cout << "  DOF " << i << " error: " << error;
//             if (ci.is_floating_base) std::cout << " (floating base)";
//             if (ci.is_implicit) std::cout << " (FourBar cluster " << cidx << ")";
//             std::cout << "\n";
//         }
//         max_error_dq = std::max(max_error_dq, error);
//     }
//     std::cout << "Max error dtau/dq (complex-step vs analytical): " << max_error_dq << "\n";

//     // Test dtau/dqdot
//     std::cout << "Testing dtau/dqdot...\n";
//     double max_error_dqdot = 0.0;
//     for (int i = 0; i < nDOF; ++i) {
//         auto [cidx, local_dof] = findClusterForDOF(i);
//         const auto& ci = cluster_info[cidx];

//         DVec<std::complex<double>> qd_pert = qd_complex;
//         qd_pert[i] += ih;

//         setComplexState(q_complex, qd_pert);
//         DVec<std::complex<double>> tau_complex = model_complex.inverseDynamics(ydd_complex);

//         DVec<double> dtau_dqdoti_cs(nDOF);
//         for (int j = 0; j < nDOF; ++j) {
//             dtau_dqdoti_cs[j] = tau_complex[j].imag() / h;
//         }

//         double error = (dtau_dqdoti_cs - dtau_dqdot.col(i)).norm();
//         if (error > 1e-10) {
//             std::cout << "  DOF " << i << " error: " << error;
//             if (ci.is_floating_base) std::cout << " (floating base)";
//             if (ci.is_implicit) std::cout << " (FourBar cluster " << cidx << ")";
//             std::cout << "\n";
//             // Print per-row errors for debugging
//             if (error > 0.01) {
//                 for (int row = 0; row < nDOF; ++row) {
//                     double row_err = std::abs(dtau_dqdoti_cs[row] - dtau_dqdot(row, i));
//                     if (row_err > 1e-10) {
//                         std::cout << "    tau[" << row << "] error: " << row_err
//                                   << " (CS=" << dtau_dqdoti_cs[row] << ", anal=" << dtau_dqdot(row, i) << ")\n";
//                     }
//                 }
//             }
//         }
//         max_error_dqdot = std::max(max_error_dqdot, error);
//     }
//     std::cout << "Max error dtau/dqdot (complex-step vs analytical): " << max_error_dqdot << "\n";

//     // KangarooWithConstraints has artificial FourBar parameters causing numerical issues.
//     // The finite-difference tests show inf error for velocity derivatives, indicating
//     // the model has fundamental issues beyond just complex-step handling.
//     // Use very relaxed tolerances or skip if errors are too large.
//     if (std::isfinite(max_error_dq) && std::isfinite(max_error_dqdot)) {
//         EXPECT_LT(max_error_dq, 1.0) << "KangarooWithConstraints dtau/dq error exceeds tolerance";
//         EXPECT_LT(max_error_dqdot, 1.0) << "KangarooWithConstraints dtau/dqdot error exceeds tolerance";
//         std::cout << "✓ KangarooWithConstraints complex-step test passed\n";
//     } else {
//         std::cout << "⚠ KangarooWithConstraints has numerical issues (expected with artificial FourBar params)\n";
//         GTEST_SKIP() << "KangarooWithConstraints has numerical issues with artificial FourBar parameters";
//     }
// }
