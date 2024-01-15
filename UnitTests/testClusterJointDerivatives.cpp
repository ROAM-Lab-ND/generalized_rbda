#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

using namespace grbda;

namespace biasVelocityTestHelpers
{

    casadi::Function
    createBiasVelocityCasadiFunction(std::shared_ptr<ClusterJoints::Base<casadi::SX>> joint)
    {
        using SX = casadi::SX;

        // Define symbolic variables
        SX cs_q_sym = SX::sym("q", joint->numPositions(), 1); // manifold space
        DVec<SX> q_sym(joint->numPositions());
        casadi::copy(cs_q_sym, q_sym);

        SX cs_dq_sym = SX::sym("dq", joint->numVelocities(), 1); // tangent space
        DVec<SX> dq_sym(joint->numVelocities());
        casadi::copy(cs_dq_sym, dq_sym);

        SX cs_qd_sym = SX::sym("qd", joint->numVelocities(), 1); // joint velocity
        DVec<SX> qd_sym(joint->numVelocities());
        casadi::copy(cs_qd_sym, qd_sym);

        // Set state and update kinematics
        JointState<SX> joint_state(JointCoordinate<SX>(q_sym, false),
                                   JointCoordinate<SX>(qd_sym, false));
        joint_state.position = TestHelpers::plus(joint->type(), q_sym, dq_sym);
        joint->updateKinematics(joint_state);

        // Differentiate the motion subspace matrix with repsect to q, ∂S/∂dq
        DMat<SX> S = joint->S();
        SX cs_S = casadi::SX(casadi::Sparsity::dense(S.rows(), S.cols()));
        casadi::copy(S, cs_S);

        SX cs_dS_ddq = jacobian(cs_S, cs_dq_sym);
        DMat<SX> dS_ddq(cs_dS_ddq.size1(), cs_dS_ddq.size2());
        casadi::copy(cs_dS_ddq, dS_ddq);

        // Tensor multiplication: dS/dt = Sring = ∂S/∂dq * dq/dt
        DMat<SX> Sring(S.rows(), S.cols());
        for (int i = 0; i < S.cols(); ++i)
        {
            Sring.col(i) = dS_ddq.middleRows(i * S.rows(), S.rows()) * qd_sym;
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
                                        casadi::SXVector{cs_q_sym, cs_dq_sym, cs_qd_sym},
                                        casadi::SXVector{cs_Sring_dq, cs_cJ});

        return csBiasVelocity;
    }

    bool biasVelocitiesAreEqual(const casadi::Function &fcn, int nq, int nv)
    {
        std::vector<casadi::DM> q = random<casadi::DM>(nq);
        std::vector<casadi::DM> dq = zeros<casadi::DM>(nv);
        std::vector<casadi::DM> qd = random<casadi::DM>(nv);
        casadi::DMVector res = fcn(casadi::DMVector{q, dq, qd});

        DVec<double> Sring_qd_full(res[0].size1());
        casadi::copy(res[0], Sring_qd_full);

        DVec<double> cJ_full(res[1].size1());
        casadi::copy(res[1], cJ_full);

        return (Sring_qd_full - cJ_full).norm() < 1e-12;
    }

}

GTEST_TEST(ClusterJointDerivatives, BiasVelocities)
{
    // This test validates that the bias velocities for each of the cluster joints is equal to the
    // time derivative of the motion subspace matrix computed via autodiff

    using namespace biasVelocityTestHelpers;
    using SX = casadi::SX;

    const int joint_samples = 10;
    const int state_samples = 10;

    // Free Cluster Joint
    for (int i = 0; i < joint_samples; i++)
    {
        using JointType = ClusterJoints::Free<SX>;
        Body<SX> body = randomBody<SX>(0, -1, 0, 0, 0);
        std::shared_ptr<JointType> joint = std::make_shared<JointType>(body);

        casadi::Function csBiasVelocity = createBiasVelocityCasadiFunction(joint);

        for (int j = 0; j < state_samples; j++)
        {
            ASSERT_TRUE(biasVelocitiesAreEqual(csBiasVelocity, joint->numPositions(), joint->numVelocities()));
        }
    }

    // Revolute Pair with Rotor Cluster Joint
    for (int i = 0; i < joint_samples; i++)
    {
        using JointType = ClusterJoints::RevolutePairWithRotor<SX>;

        Body<SX> link1 = randomBody<SX>(0, -1, 0, 0, 0);
        Body<SX> rotor1 = randomBody<SX>(1, -1, 1, 0, 0);
        Body<SX> rotor2 = randomBody<SX>(2, -1, 2, 0, 0);
        Body<SX> link2 = randomBody<SX>(3, 0, 3, 0, 0);

        ClusterJoints::ParallelBeltTransmissionModule<1, SX> module1{
            link1, rotor1, ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
            random<SX>(), Vec1<SX>{random<SX>()}};
        ClusterJoints::ParallelBeltTransmissionModule<2, SX> module2{
            link2, rotor2, ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
            random<SX>(), Vec2<SX>{random<SX>(), random<SX>()}};

        std::shared_ptr<JointType> joint = std::make_shared<JointType>(module1, module2);

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
    //         ori::randomCoordinateAxis(), ori::randomCoordinateAxis(), random<SX>()};

    //     std::shared_ptr<ClusterJoints::TelloHipDifferential<SX>> joint = std::make_shared<ClusterJoints::TelloHipDifferential<SX>>(module);

    //     casadi::Function csBiasVelocity = createBiasVelocityCasadiFunction(joint);

    //     // Validate that cJ is equal to the derivative of S * qd with respect to q
    //     for (int j = 0; j < state_samples; j++)
    //     {
    //         ASSERT_TRUE(biasVelocitiesAreEqual(csBiasVelocity, joint->numPositions(), joint->numVelocities()));
    //     }
    // }
}
