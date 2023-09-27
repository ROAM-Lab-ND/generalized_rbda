#include "gtest/gtest.h"

#include "Utils/math.h"
#include "Utils/SpatialInertia.h"
#include "Utils/SpatialTransforms.h"
#include "Dynamics/ClusterJoints/ClusterJointTypes.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <casadi/casadi.hpp>

using namespace grbda;

// TODO(@MatthewChignoli): Change the name of this file?

GTEST_TEST(Derivatives, TestQuaternionIntegration)
{
    // This test validates that quaternion integration for scalar types double and casadi::SX
    // agree with each other

    using SX = casadi::SX;

    // Create symbolic function for integrating quaternions
    SX cs_q_sym = SX::sym("q", 4, 1);
    SX cs_w_sym = SX::sym("w", 3, 1);
    SX dt_sym = SX::sym("dt");

    Quat<SX> q_sym;
    casadi::copy(cs_q_sym, q_sym);

    Vec3<SX> w_sym;
    casadi::copy(cs_w_sym, w_sym);

    Quat<SX> q_plus_sym = ori::integrateQuat(q_sym, w_sym, dt_sym);
    SX cs_q_plus_sym = casadi::SX(casadi::Sparsity::dense(q_plus_sym.rows(), 1));
    casadi::copy(q_plus_sym, cs_q_plus_sym);

    casadi::Function csIntegrateQuat("integrateQuat",
                                     casadi::SXVector{cs_q_sym, cs_w_sym, dt_sym},
                                     casadi::SXVector{cs_q_plus_sym});

    // Compare CasADi function to original quaternion integration function
    const double dt = 0.001;
    std::vector<Quat<double>> q_samples;
    std::vector<Vec3<double>> w_samples;

    for (int i = 0; i < 20; ++i)
    {
        q_samples.push_back(ori::rpyToQuat(Vec3<double>::Random()));
        w_samples.push_back(Vec3<double>::Random());

        q_samples.push_back(ori::rpyToQuat(Vec3<double>::Random()));
        w_samples.push_back(Vec3<double>::Zero());

        q_samples.push_back(ori::rpyToQuat(Vec3<double>::Zero()));
        w_samples.push_back(Vec3<double>::Random());
    }

    q_samples.push_back(ori::rpyToQuat(Vec3<double>::Zero()));
    w_samples.push_back(Vec3<double>::Zero());

    for (int i = 0; i < q_samples.size(); ++i)
    {
        Quat<double> q = q_samples[i];
        Vec3<double> w = w_samples[i];

        Quat<double> q_plus1 = ori::integrateQuat(q, w, dt);

        std::vector<double> q_vec{q[0], q[1], q[2], q[3]};
        std::vector<double> w_vec{w(0), w(1), w(2)};
        casadi::DMVector res = csIntegrateQuat(casadi::DMVector{q_vec, w_vec, dt});
        Quat<double> q_plus2;
        casadi::copy(res[0], q_plus2);

        GTEST_ASSERT_LE((q_plus1 - q_plus2).norm(), 1e-12);
    }
}

GTEST_TEST(Derivatives, TestRotationMatrix)
{
    // This test validates that conversion from rotation matrix to quaternion for scalar types
    // double and casadi::SX agree with each other

    using SX = casadi::SX;

    // Create symbolic function for integrating quaternions
    SX cs_rpy_sym = SX::sym("rpy", 3, 1);
    Vec3<SX> rpy_sym;
    casadi::copy(cs_rpy_sym, rpy_sym);
    Mat3<SX> R_sym = ori::rpyToRotMat(rpy_sym);
    Quat<SX> q_sym = ori::rotationMatrixToQuaternion(R_sym);

    SX cs_q_sym = casadi::SX(casadi::Sparsity::dense(q_sym.rows(), 1));
    casadi::copy(q_sym, cs_q_sym);

    casadi::Function csRpyToQuat("rpyToQuat",
                                 casadi::SXVector{cs_rpy_sym},
                                 casadi::SXVector{cs_q_sym});

    // Compare CasADi function to original rotation matrix to quaternion function
    for (int i = 0; i < 50; ++i)
    {
        Vec3<double> rpy = Vec3<double>::Random();
        Mat3<double> R = ori::rpyToRotMat(rpy);
        Quat<double> q1 = ori::rotationMatrixToQuaternion(R);

        std::vector<double> rpy_vec{rpy(0), rpy(1), rpy(2)};
        casadi::DMVector res = csRpyToQuat(casadi::DMVector{rpy_vec});
        Quat<double> q2;
        casadi::copy(res[0], q2);

        GTEST_ASSERT_LE((q1 - q2).norm(), 1e-12);
    }
}

GTEST_TEST(Derivatives, SpatialMotionTransform)
{
    // Compute

    // TODO(@MatthewChignoli): template the test on SX and MX?
    using SX = casadi::SX;

    // So I think that if we have a revolute Z joint, then the partial with repsect to q of Xup should be [0;0;1;0;0;0]

    SX cs_q = SX::sym("q");
    spatial::Transform<SX> XJ = spatial::jointXform(spatial::JointType::Revolute,
                                                    ori::CoordinateAxis::Z, cs_q);

    SVec<SX> v1;
    for (int i = 0; i < 6; ++i)
    {
        v1(i) = 1.;
    }

    SVec<SX> v2 = XJ.transformMotionVector(v1);

    // // Copy the result from Eigen matrices to casadi matrix
    casadi::SX cs_v2 = casadi::SX(casadi::Sparsity::dense(v2.rows(), 1));
    casadi::copy(v2, cs_v2);

    // Display the resulting casadi matrix
    std::cout << "v2 = " << cs_v2 << std::endl;

    // Do some AD
    casadi::SX dv2_dq = jacobian(cs_v2, cs_q);

    // Display the resulting jacobian
    std::cout << "dv2/dq = " << dv2_dq << std::endl;

    // Create a function
    casadi::Function fun("fun", casadi::SXVector{cs_q}, casadi::SXVector{cs_v2, dv2_dq});
    std::cout << "fun = " << fun << std::endl;

    // Evaluate the function
    casadi::DMVector res = fun(casadi::DMVector{std::vector<double>{1.}});
    std::cout << "fun(q)=" << res << std::endl;

    // Compare to finite difference
    double eps = 1e-6;
    casadi::DMVector res_plus = fun(casadi::DMVector{std::vector<double>{1. + eps}});
    // std::cout << "fun(q + eps)=" << res_plus << std::endl;

    casadi::DMVector res_minus = fun(casadi::DMVector{std::vector<double>{1. - eps}});
    // std::cout << "fun(q - eps)=" << res_minus << std::endl;

    casadi::DM res_fd = (res_plus[0] - res_minus[0]) / (2. * eps);

    std::cout << "fun(q + eps) - fun(q - eps) / (2 * eps) = " << res_fd << std::endl;
}

GTEST_TEST(Derivatives, MotionSubspaceApparentDerivative)
{
    // TODO(@MatthewChignoli): Test description
    // TODO(@MatthewChignoli): Add more randomness to this test
    // TODO(@MatthewChignoli): Template the test on the types of cluster joints
    // TODO(@MatthewChignoli): Add more comments to explain what is happening, Maybe even write some nicer wrappers for making/working with these symbolic functions

    using SX = casadi::SX;

    // Create symbolic function for computing the apparent derivative of the motion subspace
    Mat3<SX> I3 = Mat3<SX>::Identity();
    Vec3<SX> r = Vec3<SX>::Random();

    Body<SX> link1{1, "link1", 0, spatial::Transform<SX>(I3, r), SpatialInertia<SX>(), 0, 0, 0};
    Body<SX> rotor1{2, "rotor1", 0, spatial::Transform<SX>(I3, r), SpatialInertia<SX>(), 1, 0, 0};
    Body<SX> rotor2{3, "rotor2", 0, spatial::Transform<SX>(I3, r), SpatialInertia<SX>(), 2, 0, 0};
    Body<SX> link2{4, "link2", 1, spatial::Transform<SX>(I3, r), SpatialInertia<SX>(), 4, 0, 0};

    ClusterJoints::ParallelBeltTransmissionModule<SX> module1{link1, rotor1, ori::CoordinateAxis::Z, ori::CoordinateAxis::Z, 1., 1.};
    ClusterJoints::ParallelBeltTransmissionModule<SX> module2{link2, rotor2, ori::CoordinateAxis::Z, ori::CoordinateAxis::Z, 1., 1.};

    ClusterJoints::RevolutePairWithRotor<SX> joint{module1, module2};

    SX cs_q_sym = SX::sym("q", joint.numPositions(), 1);
    DVec<SX> q_sym(joint.numPositions());
    casadi::copy(cs_q_sym, q_sym);

    SX cs_qd_sym = SX::sym("qd", joint.numVelocities(), 1);
    DVec<SX> qd_sym(joint.numVelocities());
    casadi::copy(cs_qd_sym, qd_sym);

    JointState<SX> joint_state(JointCoordinate<SX>(q_sym, false),
                               JointCoordinate<SX>(qd_sym, false));

    joint.updateKinematics(joint_state);

    DVec<SX> S_qd = joint.S() * qd_sym;
    SX cs_S_qd = casadi::SX(casadi::Sparsity::dense(S_qd.rows(), 1));
    casadi::copy(S_qd, cs_S_qd);
    SX cs_Sring_qd = jacobian(cs_S_qd, cs_q_sym);

    DVec<SX> cJ = joint.cJ();
    SX cs_cJ = casadi::SX(casadi::Sparsity::dense(cJ.rows(), 1));
    casadi::copy(cJ, cs_cJ);

    casadi::Function csApparentDerivative("apparentDerivative",
                                          casadi::SXVector{cs_q_sym, cs_qd_sym},
                                          casadi::SXVector{cs_Sring_qd, cs_cJ});

    // Validate that cJ is equal to the derivative of S * qd with respect to q
    casadi::DMVector res = csApparentDerivative(casadi::DMVector{std::vector<double>{1., 1.},
                                                                 std::vector<double>{1., 1.}});

    DVec<double> Sring_qd_full(S_qd.rows());
    casadi::copy(res[0], Sring_qd_full);

    DVec<double> cJ_full(cJ.rows());
    casadi::copy(res[1], cJ_full);

    GTEST_ASSERT_LE((Sring_qd_full - cJ_full).norm(), 1e-12);
}
