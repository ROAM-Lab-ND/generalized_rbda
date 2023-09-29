#include "gtest/gtest.h"

#include "Utils/math.h"
#include "Utils/SpatialInertia.h"
#include "Utils/SpatialTransforms.h"
#include "Dynamics/ClusterJoints/ClusterJointTypes.h"
#include "Robots/RobotTypes.h"

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

namespace biasVelocityTestHelpers
{

    casadi::Function
    createBiasVelocityCasadiFunction(std::shared_ptr<ClusterJoints::Base<casadi::SX>> joint)
    {
        using SX = casadi::SX;

        // Define symbolic variables
        SX cs_q_sym = SX::sym("q", joint->numPositions(), 1);
        DVec<SX> q_sym(joint->numPositions());
        casadi::copy(cs_q_sym, q_sym);

        SX cs_qd_sym = SX::sym("qd", joint->numVelocities(), 1);
        DVec<SX> qd_sym(joint->numVelocities());
        casadi::copy(cs_qd_sym, qd_sym);

        // Set state and update kinematics
        JointState<SX> joint_state(JointCoordinate<SX>(q_sym, false),
                                   JointCoordinate<SX>(qd_sym, false));
        joint->updateKinematics(joint_state);

        // Differentiate the motion subspace matrix with repsect to q
        DMat<SX> S = joint->S();
        SX cs_S = casadi::SX(casadi::Sparsity::dense(S.rows(), S.cols()));
        casadi::copy(S, cs_S);

        SX cs_dS_dq = jacobian(cs_S, cs_q_sym);
        DMat<SX> dS_dq(cs_dS_dq.size1(), cs_dS_dq.size2());
        casadi::copy(cs_dS_dq, dS_dq);

        // Tensor multiplication: dS_dt = Sring = dS_dq * qd_sym
        DMat<SX> Sring(S.rows(), S.cols());
        for (int i = 0; i < S.cols(); ++i)
        {
            Sring.col(i) = dS_dq.middleRows(i * S.rows(), S.rows()) * qd_sym;
        }

        // Compute bias velocity from partial of motion subspace matrix
        DVec<SX> Sring_dq = Sring * qd_sym;
        SX cs_Sring_dq = casadi::SX(casadi::Sparsity::dense(S.rows(), 1));
        casadi::copy(Sring_dq, cs_Sring_dq);

        // Get bias velocity directly
        DVec<SX> cJ = joint->cJ();
        SX cs_cJ = casadi::SX(casadi::Sparsity::dense(cJ.rows(), 1));
        casadi::copy(cJ, cs_cJ);

        // Create the function
        casadi::Function csBiasVelocity("biasVelocity",
                                        casadi::SXVector{cs_q_sym, cs_qd_sym},
                                        casadi::SXVector{cs_Sring_dq, cs_cJ});

        return csBiasVelocity;
    }

    bool biasVelocitiesAreEqual(const casadi::Function &fcn, int nq, int nv)
    {
        std::vector<casadi::DM> q = math::random<casadi::DM>(nq);
        std::vector<casadi::DM> qd = math::random<casadi::DM>(nv);
        casadi::DMVector res = fcn(casadi::DMVector{q, qd});

        DVec<double> Sring_qd_full(res[0].size1());
        casadi::copy(res[0], Sring_qd_full);

        DVec<double> cJ_full(res[1].size1());
        casadi::copy(res[1], cJ_full);

        return (Sring_qd_full - cJ_full).norm() < 1e-12;
    }

}

GTEST_TEST(Derivatives, BiasVelocities)
{
    // This test validates that the bias velocities for each of the cluster joints is equal to the
    // time derivative of the motion subspace matrix

    using namespace biasVelocityTestHelpers;
    using SX = casadi::SX;

    const int joint_samples = 10;
    const int state_samples = 10;

    // Revolute Pair with Rotor Cluster Joint
    for (int i = 0; i < joint_samples; i++)
    {
        Body<SX> link1 = randomBody<SX>(1, 0, 0, 0, 0);
        Body<SX> rotor1 = randomBody<SX>(2, 0, 1, 0, 0);
        Body<SX> rotor2 = randomBody<SX>(3, 0, 2, 0, 0);
        Body<SX> link2 = randomBody<SX>(4, 1, 3, 0, 0);

        ClusterJoints::ParallelBeltTransmissionModule<SX> module1{
            link1, rotor1, ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
            math::random<SX>(), math::random<SX>()};
        ClusterJoints::ParallelBeltTransmissionModule<SX> module2{
            link2, rotor2, ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
            math::random<SX>(), math::random<SX>()};

        std::shared_ptr<ClusterJoints::RevolutePairWithRotor<SX>> joint = std::make_shared<ClusterJoints::RevolutePairWithRotor<SX>>(module1, module2);

        casadi::Function csBiasVelocity = createBiasVelocityCasadiFunction(joint);

        for (int j = 0; j < state_samples; j++)
        {
            ASSERT_TRUE(biasVelocitiesAreEqual(csBiasVelocity, joint->numPositions(), joint->numVelocities()));
        }
    }

    // TODO(@nicholasadr): add this joint to the unit test
    // Tello Differential Cluster Joint
    // for (int i = 0; i < joint_samples; i++)
    // {
    //     Body<SX> rotor1 = randomBody<SX>(1, 0, 0, 0, 0);
    //     Body<SX> rotor2 = randomBody<SX>(2, 0, 1, 0, 0);
    //     Body<SX> link1 = randomBody<SX>(3, 0, 2, 0, 0);
    //     Body<SX> link2 = randomBody<SX>(4, 3, 3, 0, 0);

    //     ClusterJoints::TelloDifferentialModule<SX> module{
    //         rotor1, rotor2, link1, link2,
    //         ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
    //         ori::randomCoordinateAxis(), ori::randomCoordinateAxis(), math::random<SX>()};

    //     std::shared_ptr<ClusterJoints::TelloHipDifferential<SX>> joint = std::make_shared<ClusterJoints::TelloHipDifferential<SX>>(module);

    //     casadi::Function csBiasVelocity = createBiasVelocityCasadiFunction(joint);

    //     // Validate that cJ is equal to the derivative of S * qd with respect to q
    //     for (int j = 0; j < state_samples; j++)
    //     {
    //         ASSERT_TRUE(biasVelocitiesAreEqual(csBiasVelocity, joint->numPositions(), joint->numVelocities()));
    //     }
    // }
}

template <class T>
class AutoDiffRobotTest : public testing::Test
{
    typedef casadi::SX SX;

protected:
    AutoDiffRobotTest() : robot_(T())
    {
        for (int i = 0; i < num_robots_; i++)
        {
            T robot;
            ClusterTreeModel<SX> model = robot.buildClusterTreeModel();

            createContactJacobianCasadiFunctions(model);
        }
    }

    ModelState<SX> createSymbolicModelState(const ClusterTreeModel<SX> &model,
                                            SX &cs_q_sym, SX &cs_qd_sym) const
    {
        DVec<SX> q_sym(model.getNumPositions());
        casadi::copy(cs_q_sym, q_sym);

        DVec<SX> qd_sym(model.getNumDegreesOfFreedom());
        casadi::copy(cs_qd_sym, qd_sym);

        ModelState<SX> state;
        for (const auto cluster : model.clusters())
        {
            DVec<SX> q_cluster = q_sym.segment(cluster->position_index_,
                                               cluster->num_positions_);
            DVec<SX> qd_cluster = qd_sym.segment(cluster->velocity_index_,
                                                 cluster->num_velocities_);
            state.push_back(JointState<SX>(JointCoordinate<SX>(q_cluster, false),
                                           JointCoordinate<SX>(qd_cluster, false)));
        }

        return state;
    }

    void createContactJacobianCasadiFunctions(ClusterTreeModel<casadi::SX> model)
    {
        SX cs_q_sym = SX::sym("q", model.getNumPositions(), 1);
        SX cs_qd_sym = SX::zeros(model.getNumDegreesOfFreedom(), 1);
        ModelState<SX> state = createSymbolicModelState(model, cs_q_sym, cs_qd_sym);

        model.setState(state);
        model.forwardKinematicsIncludingContactPoints();
        model.updateContactPointJacobians();

        std::unordered_map<std::string, casadi::Function> contact_jacobian_fcns;
        for (const ContactPoint<SX> &contact_point : model.contactPoints())
        {
            Vec3<SX> contact_point_pos = contact_point.position_;
            D3Mat<SX> contact_point_jac = contact_point.jacobian_.bottomRows<3>();

            SX cs_contact_point_pos = SX(casadi::Sparsity::dense(3, 1));
            casadi::copy(contact_point_pos, cs_contact_point_pos);

            SX cs_contact_point_jac = SX(casadi::Sparsity::dense(contact_point_jac.rows(),
                                                                 model.getNumDegreesOfFreedom()));
            casadi::copy(contact_point_jac, cs_contact_point_jac);

            SX dpos_dq = jacobian(cs_contact_point_pos, cs_q_sym);

            std::vector<SX> args{cs_q_sym};
            std::vector<SX> res{dpos_dq, cs_contact_point_jac};
            casadi::Function contactPointFunction("contactPointPositionJacobian", args, res);

            contact_jacobian_fcns[contact_point.name_] = contactPointFunction;
        }
        contact_jacobian_fcn_maps_.push_back(contact_jacobian_fcns);
    }

    T robot_;
    const int num_robots_ = 10;
    std::vector<std::unordered_map<std::string, casadi::Function>> contact_jacobian_fcn_maps_;
};

using testing::Types;

// TODO(@MatthewChignoli): Add floating base robots once we know how to handle differentiation on non-Euclidean manifolds
typedef Types<
    RevoluteChainWithRotor<2, casadi::SX>,
    RevoluteChainWithRotor<4, casadi::SX>,
    RevoluteChainWithRotor<8, casadi::SX>>
    Robots;

TYPED_TEST_SUITE(AutoDiffRobotTest, Robots);

TYPED_TEST(AutoDiffRobotTest, contactJacobians)
{
    // This test validates that the partial derivative of the contact points' positions are equal to
    // the contact points' jacobians

    using DM = casadi::DM;

    for (const auto &contact_jacobian_fcns : this->contact_jacobian_fcn_maps_)
    {
        for (const auto &contact_jacobian_fcn : contact_jacobian_fcns)
        {
            casadi::Function fcn = contact_jacobian_fcn.second;

            for (int i = 0; i < 20; i++)
            {
                std::vector<DM> q = math::random<DM>(this->robot_.getNumDofs());
                std::vector<DM> res = fcn(std::vector<DM>{q});

                DMat<double> dpos_dq_full(3, this->robot_.getNumDofs());
                casadi::copy(res[0], dpos_dq_full);

                DMat<double> contact_point_jac_full(3, this->robot_.getNumDofs());
                casadi::copy(res[1], contact_point_jac_full);

                GTEST_ASSERT_LE((dpos_dq_full - contact_point_jac_full).norm(), 1e-12);
            }
        }
    }
}
