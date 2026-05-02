
// --- IMPLICIT CONSTRAINT COMPLEX-STEP TESTS ---
#include "grbda/Robots/TelloWithArms.hpp"
#include "grbda/Robots/Tello.hpp"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include "grbda/Robots/PlanarLegLinkage.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <complex>
#include <vector>
#include <string>
#include "gtest/gtest.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"
#include "testHelpers.hpp"
#include "config.h"

using namespace grbda;

// ── CSV output ──────────────────────────────────────────────────────────────

struct SummaryRow {
    std::string robot_name;
    int         dof;
    double      max_err_dq;
    double      max_err_dqdot;
};

struct PerJointRow {
    std::string robot_name;
    std::string cluster_name;
    int         joint_idx;
    double      err_dq;
    double      err_dqdot;
};

struct CsvAccumulator {
    std::vector<SummaryRow>  summary;
    std::vector<PerJointRow> per_joint;

    static CsvAccumulator& get() {
        static CsvAccumulator inst;
        return inst;
    }

    void flush() const {
        const std::string base = std::string(SOURCE_DIRECTORY) + "/Benchmarking/data/";

        std::ofstream fs(base + "accuracy_summary.csv");
        if (fs.is_open()) {
            fs << "robot_name,dof,max_err_dq,max_err_dqdot\n";
            fs << std::scientific << std::setprecision(6);
            for (const auto& r : summary)
                fs << r.robot_name << "," << r.dof << ","
                   << r.max_err_dq << "," << r.max_err_dqdot << "\n";
        }

        std::ofstream fj(base + "minicheetah_per_joint.csv");
        if (fj.is_open()) {
            fj << "robot_name,cluster_name,joint_idx,err_dq,err_dqdot\n";
            fj << std::scientific << std::setprecision(6);
            for (const auto& r : per_joint)
                fj << r.robot_name << "," << r.cluster_name << ","
                   << r.joint_idx << "," << r.err_dq << "," << r.err_dqdot << "\n";
        }
    }
};

class CsvWriteEnvironment : public ::testing::Environment {
public:
    void TearDown() override { CsvAccumulator::get().flush(); }
};

// ── Helper ───────────────────────────────────────────────────────────────────

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
    double tol_dqdot = 1e-12,
    bool record_per_joint = false) {
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


    // Record summary row
    CsvAccumulator::get().summary.push_back({robot_name, nDOF, max_error_dq, max_error_dqdot});

    // Record per-joint rows (column-wise max of the error matrix across all tau outputs)
    if (record_per_joint) {
        int dof_idx = 0;
        for (const auto& cluster : model_real.clusters()) {
            const int nv = cluster->num_velocities_;
            for (int k = 0; k < nv; ++k, ++dof_idx) {
                CsvAccumulator::get().per_joint.push_back({
                    robot_name,
                    cluster->name_,
                    dof_idx,
                    (dtau_dq    - dtau_dq_cs   ).col(dof_idx).cwiseAbs().maxCoeff(),
                    (dtau_dqdot - dtau_dqdot_cs).col(dof_idx).cwiseAbs().maxCoeff()
                });
            }
        }
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
        model_real, model_complex, "MiniCheetah (Quaternion)",
        /*tol_dq=*/1e-12, /*tol_dqdot=*/1e-12, /*record_per_joint=*/true);
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

    model_real.setState(randomModelState(model_real,true));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "MIT Humanoid (Quaternion)", 1e-12, 1e-13);
}

TEST(InverseDynamicsDerivativesComplexStep, KukaLWR) {
    KukaLWR<double>               robot_real;
    KukaLWR<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real));

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "KUKA LWR 4+");
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

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "PlanarLegLinkage (ImplicitConstraint)", 1e-12, 1e-14);
}

TEST(InverseDynamicsDerivativesComplexStep, KangarooOpenChainDerivatives) {
    Kangaroo<double>               robot_real;
    Kangaroo<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real, true), true);

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "Kangaroo (Open Chain)", 1e-12, 1e-14);
}

TEST(InverseDynamicsDerivativesComplexStep, CassieOpenChainDerivatives) {
    Cassie<double>               robot_real;
    Cassie<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real, true), true);

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "Cassie (Closed Chain)", 1e-12, 1e-14);
}

TEST(InverseDynamicsDerivativesComplexStep, KangarooWithConstraintsDerivatives) {
    KangarooWithConstraints<double>               robot_real;
    KangarooWithConstraints<std::complex<double>> robot_complex;
    ClusterTreeModel<double>               model_real    = robot_real.buildClusterTreeModel();
    ClusterTreeModel<std::complex<double>> model_complex = robot_complex.buildClusterTreeModel();

    model_real.setState(randomModelState(model_real, true), true);

    testInverseDynamicsDerivativesComplexStep(
        model_real, model_complex, "KangarooWithConstraints (Closed Chain)", 1e-12, 1e-14);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new CsvWriteEnvironment);
    return RUN_ALL_TESTS();
}
