#include "gtest/gtest.h"

#include "Utils/math.h"
#include "Utils/SpatialInertia.h"
#include "Utils/SpatialTransforms.h"
#include "Dynamics/ClusterJoints/ClusterJointTypes.h"
#include "Robots/SerialChains/RevolutePairChainWithRotor.hpp"

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

// TODO(@MatthewChignoli): Do this with full robots instead of just a single cluster joint?
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

    // TODO(@MatthewChignoli): Only looking at outputs of casadi function? Is that fair?
    // Validate that cJ is equal to the derivative of S * qd with respect to q
    casadi::DMVector res = csApparentDerivative(casadi::DMVector{std::vector<double>{1., 1.},
                                                                 std::vector<double>{1., 1.}});

    DVec<double> Sring_qd_full(S_qd.rows());
    casadi::copy(res[0], Sring_qd_full);

    DVec<double> cJ_full(cJ.rows());
    casadi::copy(res[1], cJ_full);

    GTEST_ASSERT_LE((Sring_qd_full - cJ_full).norm(), 1e-12);
}

GTEST_TEST(Derivatives, ContactJacobians)
{
    // TODO(@MatthewChignoli): How to deal with differentiation on non-Euclidean manifolds

    using SX = casadi::SX;
    using Sparsity = casadi::Sparsity;
    using SXVector = casadi::SXVector;
    using DMVector = casadi::DMVector;

    RevolutePairChainWithRotor<2, SX> robot;
    ClusterTreeModel<SX> model = robot.buildClusterTreeModel();

    // Create function to symbolically compute the jacobian of the contact point position
    SX cs_q_sym = SX::sym("q", robot.getNumDofs(), 1);
    DVec<SX> q_sym(robot.getNumDofs());
    casadi::copy(cs_q_sym, q_sym);

    SX cs_qd_sym = SX::zeros(robot.getNumDofs(), 1);
    DVec<SX> qd_sym(robot.getNumDofs());
    casadi::copy(cs_qd_sym, qd_sym);

    ModelState<SX> state;
    state.push_back(JointState<SX>(JointCoordinate<SX>(q_sym, false),
                                   JointCoordinate<SX>(qd_sym, false)));

    model.setState(state);
    model.forwardKinematicsIncludingContactPoints();
    model.updateContactPointJacobians();

    const ContactPoint<SX> contact_point = model.contactPoints()[0];
    Vec3<SX> contact_point_pos = contact_point.position_;
    D3Mat<SX> contact_point_jac = contact_point.jacobian_.bottomRows<3>();

    SX cs_contact_point_pos = SX(Sparsity::dense(3, 1));
    casadi::copy(contact_point_pos, cs_contact_point_pos);

    SX cs_contact_point_jac = SX(Sparsity::dense(contact_point_jac.rows(), robot.getNumDofs()));
    casadi::copy(contact_point_jac, cs_contact_point_jac);

    SX dpos_dq = jacobian(cs_contact_point_pos, cs_q_sym);

    casadi::Function csContactPointPositionJacobian("contactPointPositionJacobian",
                                                    SXVector{cs_q_sym},
                                                    SXVector{dpos_dq, cs_contact_point_jac});

    // TODO(@MatthewChignoli): Better description
    // Validate that the jacobian of the contact point position is equal to the jacobian of the contact point position
    DMVector res = csContactPointPositionJacobian(DMVector{std::vector<double>{1., 1.}});

    DMat<double> dpos_dq_full(3, robot.getNumDofs());
    casadi::copy(res[0], dpos_dq_full);

    DMat<double> contact_point_jac_full(3, robot.getNumDofs());
    casadi::copy(res[1], contact_point_jac_full);

    GTEST_ASSERT_LE((dpos_dq_full - contact_point_jac_full).norm(), 1e-12);
}
