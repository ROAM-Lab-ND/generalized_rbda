#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"
#include "Robots/RobotTypes.h"

using namespace grbda;

static const double tol = 1e-10;
static const double loose_tol = 1e-3;

template <class T>
class RigidBodyKinemaitcsTest : public testing::Test
{
protected:
    RigidBodyKinemaitcsTest() : cluster_model(robot.buildClusterTreeModel()),
                                generic_model(extractGenericJointModel(cluster_model)),
                                rigid_body_model(cluster_model) {}

    bool initializeRandomStates()
    {
        model_state.clear();
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
        for (const auto &cluster : cluster_model.clusters())
        {
            JointState joint_state = cluster->joint_->randomJointState();
            JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                   spanning_joint_state.position);
            spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                   spanning_joint_state.velocity);
            model_state.push_back(joint_state);
        }

        cluster_model.initializeState(model_state);
        generic_model.initializeState(model_state);
        rigid_body_model.initializeState(spanning_joint_pos, spanning_joint_vel);

        // Check for NaNs
        bool nan_detected = false;
        for (const auto &cluster : this->cluster_model.clusters())
        {
            if (cluster->joint_state_.position.hasNaN())
            {
                nan_detected = true;
                break;
            }
        }
        return nan_detected;
    }

    T robot;
    ClusterTreeModel cluster_model;
    ClusterTreeModel generic_model;
    RigidBodyTreeModel rigid_body_model;

    ModelState model_state;
};

using testing::Types;

typedef Types<
    RevoluteChainWithRotor<2>,
    RevoluteChainWithRotor<4>,
    RevolutePairChain<2>,
    RevolutePairChain<4>,
    RevolutePairChainWithRotor<2>,
    RevolutePairChainWithRotor<4>,
    RevoluteTripleChainWithRotor<3>,
    RevoluteTripleChainWithRotor<6>,
    RevoluteChainMultipleRotorsPerLink<2, 2>,
    RevoluteChainMultipleRotorsPerLink<4, 1>,
    RevoluteChainMultipleRotorsPerLink<4, 3>,
    RevoluteChainWithAndWithoutRotor<0ul, 8ul>,
    RevoluteChainWithAndWithoutRotor<4ul, 4ul>,
    RevoluteChainWithAndWithoutRotor<8ul, 0ul>,
    Tello, TeleopArm, MIT_Humanoid, MiniCheetah>
    Robots;

TYPED_TEST_SUITE(RigidBodyKinemaitcsTest, Robots);

TYPED_TEST(RigidBodyKinemaitcsTest, ForwardKinematics)
{
    // This test compares the forward kinematics of a robot (rigid-body velocities and
    // contact-point positions/velocities) as computed by the cluster tree model versus the
    // rigid-body tree model.

    for (int k = 0; k < 20; k++)
    {
        // Initialize random state
        bool nan_detected_in_state = this->initializeRandomStates();
        if (nan_detected_in_state)
        {
            continue;
        }

        // Forward kinematics
        this->cluster_model.forwardKinematics();
        this->generic_model.forwardKinematics();
        this->rigid_body_model.forwardKinematics();

        // Verify link kinematics
        for (const auto& body : this->cluster_model.bodies())
        {
            const Vec3<double> p_cluster = this->cluster_model.getPosition(body.name_);
            const Vec3<double> p_generic = this->generic_model.getPosition(body.name_);
            const Vec3<double> p_rigid_body = this->rigid_body_model.getPosition(body.name_);
            GTEST_ASSERT_LT((p_cluster - p_generic).norm(), tol);
            GTEST_ASSERT_LT((p_cluster - p_rigid_body).norm(), tol);

            const Mat3<double> R_cluster = this->cluster_model.getOrientation(body.name_);
            const Mat3<double> R_generic = this->generic_model.getOrientation(body.name_);
            const Mat3<double> R_rigid_body = this->rigid_body_model.getOrientation(body.name_);
            GTEST_ASSERT_LT((R_cluster - R_generic).norm(), tol);
            GTEST_ASSERT_LT((R_cluster - R_rigid_body).norm(), tol);

            const Vec3<double> v_cluster = this->cluster_model.getLinearVelocity(body.name_);
            const Vec3<double> v_generic = this->generic_model.getLinearVelocity(body.name_);
            const Vec3<double> v_rigid_body = this->rigid_body_model.getLinearVelocity(body.name_);
            GTEST_ASSERT_LT((v_cluster - v_generic).norm(), tol);
            GTEST_ASSERT_LT((v_cluster - v_rigid_body).norm(), tol);

            const Vec3<double> w_cluster = this->cluster_model.getAngularVelocity(body.name_);
            const Vec3<double> w_generic = this->generic_model.getAngularVelocity(body.name_);
            const Vec3<double> w_rigid_body = this->rigid_body_model.getAngularVelocity(body.name_);
            GTEST_ASSERT_LT((w_cluster - w_generic).norm(), tol);
            GTEST_ASSERT_LT((w_cluster - w_rigid_body).norm(), tol);
        }

        // Verify cartesian velocity of contact points
        GTEST_ASSERT_EQ(this->cluster_model.contactPoints().size(),
                        this->rigid_body_model.contactPoints().size());

        this->cluster_model.contactJacobians();
        this->rigid_body_model.contactJacobians();
        for (int j = 0; j < (int)this->cluster_model.contactPoints().size(); j++)
        {
            const ContactPoint &cluster_cp = this->cluster_model.contactPoint(j);
            const ContactPoint &rigid_body_cp = this->rigid_body_model.contactPoint(j);

            // Verify positions
            const Vec3<double> p_cp_cluster = cluster_cp.position_;
            const Vec3<double> p_cp_rigid_body = rigid_body_cp.position_;
            GTEST_ASSERT_LT((p_cp_cluster - p_cp_rigid_body).norm(), tol);

            // Verify velocities
            const Vec3<double> v_cp_cluster = cluster_cp.velocity_;
            const Vec3<double> v_cp_rigid_body = rigid_body_cp.velocity_;
            GTEST_ASSERT_LT((v_cp_cluster - v_cp_rigid_body).norm(), tol);

            // Verify jacobians
            const D6Mat<double> J_cp_cluster = cluster_cp.jacobian_;
            const D6Mat<double> J_cp_rigid_body = rigid_body_cp.jacobian_;
            GTEST_ASSERT_LT((J_cp_cluster - J_cp_rigid_body).norm(), tol);

            // Verify that jacobians produce the same cartesian velocity
            Vec6<double> J_qdot = Vec6<double>::Zero();
            for (size_t i = 0; i < this->cluster_model.clusters().size(); i++)
            {
                const auto cluster = this->cluster_model.cluster(i);
                const int &vel_idx = cluster->velocity_index_;
                const int &num_vel = cluster->num_velocities_;
                J_qdot += J_cp_cluster.middleCols(vel_idx, num_vel) * this->model_state[i].velocity;
            }
            GTEST_ASSERT_LT((J_qdot.tail<3>() - v_cp_cluster).norm(), tol);
        }
    }
}

TYPED_TEST(RigidBodyKinemaitcsTest, MotionSubspaceApparentDerivative)
{
    // This test compares the apparent derivative of the motion subspace (S_ring) as computed by
    // the cluster tree model to the apparent derivative as computed by finite difference

    double dt = 0.00001;
    const int nq = this->cluster_model.getNumPositions();
    const int nv = this->cluster_model.getNumDegreesOfFreedom();

    for (int k = 0; k < 5; k++)
    {
        bool nan_detected_in_state = this->initializeRandomStates();
        if (nan_detected_in_state)
        {
            continue;
        }
        this->cluster_model.forwardKinematics();

        for (auto &cluster : this->cluster_model.clusters())
        {
            auto joint = cluster->joint_;
            JointState joint_state = cluster->joint_state_;

            const DVec<double> cJ = joint->cJ();

            JointState q_plus_joint_state = cluster->joint_state_;
            q_plus_joint_state.position = cluster->integratePosition(joint_state, dt);
            q_plus_joint_state.velocity = cluster->jointVelocity();
            joint->updateKinematics(q_plus_joint_state);
            DMat<double> S_plus = joint->S();

            JointState q_minus_joint_state = cluster->joint_state_;
            q_minus_joint_state.position = cluster->integratePosition(joint_state, -dt);
            q_minus_joint_state.velocity = cluster->jointVelocity();
            joint->updateKinematics(q_minus_joint_state);
            DMat<double> S_minus = joint->S();

            DMat<double> S_ring_fd = (S_plus - S_minus) / (2 * dt);
            DVec<double> cJ_fd = S_ring_fd * joint_state.velocity;

            GTEST_ASSERT_LT((cJ - cJ_fd).norm(), loose_tol);
        }
    }
}
