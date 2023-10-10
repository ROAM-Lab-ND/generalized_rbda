#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "Utils/Utilities.h"
#include "Utils/SpatialInertia.h"
#include "Utils/SpatialTransforms.h"
#include "Dynamics/ClusterJoints/ClusterJointTypes.h"
#include "Robots/RobotTypes.h"

#include <casadi/casadi.hpp>

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

GTEST_TEST(Derivatives, BiasVelocities)
{
    // This test validates that the bias velocities for each of the cluster joints is equal to the
    // time derivative of the motion subspace matrix

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

        ClusterJoints::ParallelBeltTransmissionModule<SX> module1{
            link1, rotor1, ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
            random<SX>(), random<SX>()};
        ClusterJoints::ParallelBeltTransmissionModule<SX> module2{
            link2, rotor2, ori::randomCoordinateAxis(), ori::randomCoordinateAxis(),
            random<SX>(), random<SX>()};

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

template <class T>
class AutoDiffRobotTest : public testing::Test
{
    typedef casadi::SX SX;

protected:
    AutoDiffRobotTest()
    {
        for (int i = 0; i < num_robots_; i++)
        {

            T robot;
            ClusterTreeModel<SX> model = robot.buildClusterTreeModel();

            createContactJacobianCasadiFunctions(model);
            createRneaCasadiFunctions(model);

            if (i == 0)
            {
                nq_ = model.getNumPositions();
                nv_ = model.getNumDegreesOfFreedom();
            }
        }
    }

    // TODO(@MatthewChignoli): Make this a more general helper function?
    ModelState<SX> createSymbolicModelState(const ClusterTreeModel<SX> &model,
                                            SX &cs_q_sym, SX &cs_dq_sym, SX &cs_qd_sym) const
    {
        DVec<SX> q_sym(model.getNumPositions());
        casadi::copy(cs_q_sym, q_sym);

        DVec<SX> dq_sym(model.getNumDegreesOfFreedom());
        casadi::copy(cs_dq_sym, dq_sym);

        DVec<SX> qd_sym(model.getNumDegreesOfFreedom());
        casadi::copy(cs_qd_sym, qd_sym);

        ModelState<SX> state;
        for (const auto cluster : model.clusters())
        {

            DVec<SX> q_cluster = q_sym.segment(cluster->position_index_, cluster->num_positions_);
            const int &vel_idx = cluster->velocity_index_;
            const int &num_vel = cluster->num_velocities_;
            DVec<SX> dq_cluster = dq_sym.segment(vel_idx, num_vel);
            DVec<SX> qd_cluster = qd_sym.segment(vel_idx, num_vel);

            JointState<SX> joint_state(JointCoordinate<SX>(q_cluster, false),
                                       JointCoordinate<SX>(qd_cluster, false));
            joint_state.position = TestHelpers::plus(cluster->joint_->type(),
                                                     q_cluster, dq_cluster);
            state.push_back(joint_state);
        }

        return state;
    }

    void createContactJacobianCasadiFunctions(ClusterTreeModel<casadi::SX> model)
    {
        typedef casadi::Sparsity Sparsity;

        SX cs_q_sym = SX::sym("q", model.getNumPositions(), 1);
        SX cs_dq_sym = SX::sym("dq", model.getNumDegreesOfFreedom(), 1);
        SX cs_qd_sym = SX::zeros(model.getNumDegreesOfFreedom(), 1);
        ModelState<SX> state = createSymbolicModelState(model, cs_q_sym, cs_dq_sym, cs_qd_sym);

        model.setState(state);
        model.forwardKinematicsIncludingContactPoints();
        model.updateContactPointJacobians();

        std::unordered_map<std::string, casadi::Function> contact_jacobian_fcns;
        for (const ContactPoint<SX> &contact_point : model.contactPoints())
        {
            Vec3<SX> contact_point_pos = contact_point.position_;
            D3Mat<SX> contact_point_jac = contact_point.jacobian_.bottomRows<3>();

            SX cs_contact_point_pos = SX(Sparsity::dense(3, 1));
            casadi::copy(contact_point_pos, cs_contact_point_pos);

            SX cs_contact_point_jac = SX(Sparsity::dense(3, model.getNumDegreesOfFreedom()));
            casadi::copy(contact_point_jac, cs_contact_point_jac);

            SX dpos_ddq = jacobian(cs_contact_point_pos, cs_dq_sym);

            std::vector<SX> args{cs_q_sym, cs_dq_sym};
            std::vector<SX> res{dpos_ddq, cs_contact_point_jac, cs_contact_point_pos};
            casadi::Function contactPointFunction("contactPointPositionJacobian", args, res);

            contact_jacobian_fcns[contact_point.name_] = contactPointFunction;
        }
        contact_jacobian_fcn_maps_.push_back(contact_jacobian_fcns);
    }

    void createRneaCasadiFunctions(ClusterTreeModel<casadi::SX> model)
    {
        typedef casadi::Sparsity Sparsity;

        SX cs_q_sym = SX::sym("q", model.getNumPositions(), 1);
        SX cs_dq_sym = SX::sym("dq", model.getNumDegreesOfFreedom(), 1);
        SX cs_qd_sym = SX::sym("qd", model.getNumDegreesOfFreedom(), 1);
        ModelState<SX> state = createSymbolicModelState(model, cs_q_sym, cs_dq_sym, cs_qd_sym);

        model.setState(state);

        SX cs_tau_sym = SX::sym("tau", model.getNumDegreesOfFreedom(), 1);
        DVec<SX> tau_sym(model.getNumDegreesOfFreedom());
        casadi::copy(cs_tau_sym, tau_sym);

        DVec<SX> qdd_sym = model.inverseDynamics(tau_sym);
        SX cs_qdd_sym = SX(Sparsity::dense(qdd_sym.size(), 1));
        casadi::copy(qdd_sym, cs_qdd_sym);

        SX dqdd_ddq = jacobian(cs_qdd_sym, cs_dq_sym);
        SX dqdd_dqd = jacobian(cs_qdd_sym, cs_qd_sym);
        SX dqdd_dtau = jacobian(cs_qdd_sym, cs_tau_sym);

        std::vector<SX> args{cs_q_sym, cs_dq_sym, cs_qd_sym, cs_tau_sym};
        std::vector<SX> res{cs_qdd_sym, dqdd_ddq, dqdd_dqd, dqdd_dtau};
        casadi::Function rneaFunction("rnea", args, res);
        rnea_fcns_.push_back(rneaFunction);
    }

    int nq_, nv_;
    const int num_robots_ = 4;
    std::vector<std::unordered_map<std::string, casadi::Function>> contact_jacobian_fcn_maps_;
    std::vector<casadi::Function> rnea_fcns_;
};

using testing::Types;

typedef Types<
    SingleRigidBody<casadi::SX>,
    MiniCheetah<casadi::SX>, MIT_Humanoid<casadi::SX>,
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

            for (int i = 0; i < 5; i++)
            {
                // Random state
                std::vector<DM> q = random<DM>(this->nq_);
                std::vector<DM> dq = zeros<DM>(this->nv_);

                // TODO(@MatthewChignoli): Fix this current method for dealing with floating base robots
                if (this->nq_ != this->nv_)
                {
                    Quat<double> quat = ori::rpyToQuat(Vec3<double>::Random());
                    q[3] = quat[0];
                    q[4] = quat[1];
                    q[5] = quat[2];
                    q[6] = quat[3];
                }

                // Compare autodiff against analytical
                std::vector<DM> res = fcn(std::vector<DM>{q, dq});

                DMat<double> dpos_ddq_full(3, this->nv_);
                casadi::copy(res[0], dpos_ddq_full);

                DMat<double> contact_point_jac_full(3, this->nv_);
                casadi::copy(res[1], contact_point_jac_full);

                GTEST_ASSERT_LE((contact_point_jac_full - dpos_ddq_full).norm(), 1e-12);

                // Compare autodiff against numerical
                DMat<double> contact_point_jac_fd(3, this->nv_);

                double h = 1e-8;
                for (int j = 0; j < this->nv_; j++)
                {
                    std::vector<DM> dq_plus = dq;
                    dq_plus[j] += h;
                    std::vector<DM> q_plus = TestHelpers::plus(q, dq_plus);

                    std::vector<DM> res_plus = fcn(std::vector<DM>{q_plus, dq});
                    DVec<double> contact_point_pos_plus(3);
                    casadi::copy(res_plus[2], contact_point_pos_plus);

                    std::vector<DM> dq_minus = dq;
                    dq_minus[j] -= h;
                    std::vector<DM> q_minus = TestHelpers::plus(q, dq_minus);

                    std::vector<DM> res_minus = fcn(std::vector<DM>{q_minus, dq});
                    DVec<double> contact_point_pos_minus(3);
                    casadi::copy(res_minus[2], contact_point_pos_minus);

                    contact_point_jac_fd.col(j) =
                        (contact_point_pos_plus - contact_point_pos_minus) / (2 * h);
                }

                GTEST_ASSERT_LE((contact_point_jac_fd - contact_point_jac_full).norm(), 1e-6);
            }
        }
    }
}

TYPED_TEST(AutoDiffRobotTest, rnea)
{
    // This test validates that the partial derivatives of the inverse dynamics with respect to q,
    // qd, and tau and the same when computed via autodiff and finite difference

    using DM = casadi::DM;

    for (const casadi::Function &fcn : this->rnea_fcns_)
    {
        for (int i = 0; i < 5; i++)
        {
            // Random state
            std::vector<DM> q = random<DM>(this->nq_);
            std::vector<DM> dq = zeros<DM>(this->nv_);
            std::vector<DM> qd = random<DM>(this->nv_);
            std::vector<DM> tau = random<DM>(this->nv_);

            // TODO(@MatthewChignoli): Fix this current method for dealing with floating base robots
            if (this->nq_ != this->nv_)
            {
                Quat<double> quat = ori::rpyToQuat(Vec3<double>::Random());
                q[3] = quat[0];
                q[4] = quat[1];
                q[5] = quat[2];
                q[6] = quat[3];
            }

            // Autodiff
            DMat<double> dqdd_ddq_ad(this->nv_, this->nv_);
            DMat<double> dqdd_dqd_ad(this->nv_, this->nv_);
            DMat<double> dqdd_dtau_ad(this->nv_, this->nv_);

            std::vector<DM> res = fcn(std::vector<DM>{q, dq, qd, tau});

            casadi::copy(res[1], dqdd_ddq_ad);
            casadi::copy(res[2], dqdd_dqd_ad);
            casadi::copy(res[3], dqdd_dtau_ad);

            // Finite Difference
            double h = 1e-8;

            // Partial with respect to dq
            DMat<double> dqdd_ddq_fd(this->nv_, this->nv_);
            for (int j = 0; j < this->nv_; j++)
            {
                std::vector<DM> dq_plus = dq;
                dq_plus[j] += h;
                std::vector<DM> q_plus = TestHelpers::plus(q, dq_plus);

                std::vector<DM> res_plus = fcn(std::vector<DM>{q_plus, dq, qd, tau});
                DVec<double> qdd_plus(this->nv_);
                casadi::copy(res_plus[0], qdd_plus);

                std::vector<DM> dq_minus = dq;
                dq_minus[j] -= h;
                std::vector<DM> q_minus = TestHelpers::plus(q, dq_minus);

                std::vector<DM> res_minus = fcn(std::vector<DM>{q_minus, dq, qd, tau});
                DVec<double> qdd_minus(this->nv_);
                casadi::copy(res_minus[0], qdd_minus);

                dqdd_ddq_fd.col(j) = (qdd_plus - qdd_minus) / (2 * h);
            }
            GTEST_ASSERT_LE((dqdd_ddq_ad - dqdd_ddq_fd).norm(), 1e-5);

            // Partial with respect to dq
            DMat<double> dqdd_dqd_fd(this->nv_, this->nv_);
            for (int j = 0; j < this->nv_; j++)
            {
                // Partial with respect to qd
                std::vector<DM> qd_plus = qd;
                qd_plus[j] += h;

                std::vector<DM> res_plus = fcn(std::vector<DM>{q, dq, qd_plus, tau});
                DVec<double> qdd_plus(this->nv_);
                casadi::copy(res_plus[0], qdd_plus);

                std::vector<DM> qd_minus = qd;
                qd_minus[j] -= h;

                std::vector<DM> res_minus = fcn(std::vector<DM>{q, dq, qd_minus, tau});
                DVec<double> qdd_minus(this->nv_);
                casadi::copy(res_minus[0], qdd_minus);

                dqdd_dqd_fd.col(j) = (qdd_plus - qdd_minus) / (2 * h);
            }
            GTEST_ASSERT_LE((dqdd_dqd_ad - dqdd_dqd_fd).norm(), 1e-5);

            // Partial with respect to tau
            DMat<double> dqdd_dtau_fd(this->nv_, this->nv_);
            for (int j = 0; j < this->nv_; j++)
            {
                // Partial with respect to tau
                std::vector<DM> tau_plus = tau;
                tau_plus[j] += h;

                std::vector<DM> res_plus = fcn(std::vector<DM>{q, dq, qd, tau_plus});
                DVec<double> qdd_plus(this->nv_);
                casadi::copy(res_plus[0], qdd_plus);

                std::vector<DM> tau_minus = tau;
                tau_minus[j] -= h;

                std::vector<DM> res_minus = fcn(std::vector<DM>{q, dq, qd, tau_minus});
                DVec<double> qdd_minus(this->nv_);
                casadi::copy(res_minus[0], qdd_minus);

                dqdd_dtau_fd.col(j) = (qdd_plus - qdd_minus) / (2 * h);
            }
            GTEST_ASSERT_LE((dqdd_dtau_ad - dqdd_dtau_fd).norm(), 1e-5);
        }
    }
}
