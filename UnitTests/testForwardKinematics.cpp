#include "gtest/gtest.h"

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"
#include "Robots/RobotTypes.h"

using namespace grbda;

static const double tol = 1e-10;

template <class T>
class RigidBodyKinemaitcsTest : public testing::Test
{
protected:
    RigidBodyKinemaitcsTest() : cluster_model(robot.buildClusterTreeModel()),
                                rigid_body_model(cluster_model) {}

    bool initializeRandomStates()
    {
        ModelState model_state;
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
    RigidBodyTreeModel rigid_body_model;
};

using testing::Types;

typedef Types<
    RevoluteChainWithRotor<2>,
    RevoluteChainWithRotor<4>,
    RevolutePairChainWithRotor<2>,
    RevolutePairChainWithRotor<4>,
    RevoluteChainMultipleRotorsPerLink<2, 2>,
    RevoluteChainMultipleRotorsPerLink<4, 1>,
    RevoluteChainMultipleRotorsPerLink<4, 3>,
    RevoluteChainWithAndWithoutRotor<0ul, 8ul>,
    RevoluteChainWithAndWithoutRotor<4ul, 4ul>,
    RevoluteChainWithAndWithoutRotor<8ul, 0ul>,
    Tello, TeleopArm>
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
        this->rigid_body_model.forwardKinematics();

        // Verify link kinematics
        for (const auto& body : this->cluster_model.bodies())
        {
            const Vec3<double> p_cluster = this->cluster_model.getPosition(body.name_);
            const Vec3<double> p_rigid_body = this->rigid_body_model.getPosition(body.name_);
            GTEST_ASSERT_LT((p_cluster - p_rigid_body).norm(), tol);

            const Mat3<double> R_cluster = this->cluster_model.getOrientation(body.name_);
            const Mat3<double> R_rigid_body = this->rigid_body_model.getOrientation(body.name_);
            GTEST_ASSERT_LT((R_cluster - R_rigid_body).norm(), tol);

            const Vec3<double> v_cluster = this->cluster_model.getLinearVelocity(body.name_);
            const Vec3<double> v_rigid_body = this->rigid_body_model.getLinearVelocity(body.name_);
            GTEST_ASSERT_LT((v_cluster - v_rigid_body).norm(), tol);

            const Vec3<double> w_cluster = this->cluster_model.getAngularVelocity(body.name_);
            const Vec3<double> w_rigid_body = this->rigid_body_model.getAngularVelocity(body.name_);
            GTEST_ASSERT_LT((w_cluster - w_rigid_body).norm(), tol);
        }

        // Verify cartesian velocity of contact points
        GTEST_ASSERT_EQ(this->cluster_model.contactPoints().size(),
                        this->rigid_body_model.contactPoints().size());

        for (int j = 0; j < (int)this->cluster_model.contactPoints().size(); j++)
        {
            // Verify positions
            Vec3<double> p_cp_cluster = this->cluster_model.contactPoint(j).position_;
            Vec3<double> p_cp_rigid_body = this->rigid_body_model.contactPoint(j).position_;
            GTEST_ASSERT_LT((p_cp_cluster - p_cp_rigid_body).norm(), tol);

            // Verify velocities
            Vec3<double> v_cp_cluster = this->cluster_model.contactPoint(j).velocity_;
            Vec3<double> v_cp_rigid_body = this->rigid_body_model.contactPoint(j).velocity_;
            GTEST_ASSERT_LT((v_cp_cluster - v_cp_rigid_body).norm(), tol);
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

            // ISSUE #14
            if (joint_state.position.size() != joint_state.velocity.size())
            {
                continue;
            }

            DMat<double> S_ring = joint->S_ring();

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

            GTEST_ASSERT_LT((S_ring - S_ring_fd).norm(), 1e-3);
        }
    }
}
