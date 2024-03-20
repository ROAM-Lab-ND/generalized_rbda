#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "grbda/Dynamics/RigidBodyTreeModel.h"
#include "grbda/Dynamics/ReflectedInertiaTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

static const double tol = 1e-10;
static const double loose_tol = 1e-3;

template <class T>
class RigidBodyKinematicsTest : public testing::Test
{
protected:
    RigidBodyKinematicsTest() : cluster_model(robot.buildClusterTreeModel()),
                                generic_model(TestHelpers::extractGenericJointModel(cluster_model)),
                                rigid_body_model(cluster_model) {}

    void initializeRandomStates()
    {
        model_state.clear();
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
        for (const auto &cluster : cluster_model.clusters())
        {
            JointState<> joint_state = cluster->joint_->randomJointState();
            JointState<> spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                   spanning_joint_state.position);
            spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                   spanning_joint_state.velocity);
            model_state.push_back(joint_state);
        }

        cluster_model.setState(model_state);
        generic_model.setState(model_state);
        rigid_body_model.setState(spanning_joint_pos, spanning_joint_vel);
    }

    T robot;
    ClusterTreeModel<> cluster_model;
    ClusterTreeModel<> generic_model;
    grbda::RigidBodyTreeModel<> rigid_body_model;

    ModelState<> model_state;
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
    RevoluteChainWithAndWithoutRotor<0ul, 8ul>,
    RevoluteChainWithAndWithoutRotor<4ul, 4ul>,
    RevoluteChainWithAndWithoutRotor<8ul, 0ul>,
    PlanarLegLinkage<>,
    Tello,
    TeleopArm,
    MIT_Humanoid<>, MIT_Humanoid<double, ori_representation::RollPitchYaw>,
    MIT_Humanoid_no_rotors<>,
    MiniCheetah<>, MiniCheetah<double, ori_representation::RollPitchYaw>>
    Robots;

TYPED_TEST_SUITE(RigidBodyKinematicsTest, Robots);

TYPED_TEST(RigidBodyKinematicsTest, ForwardKinematics)
{
    // This test compares the forward kinematics of a robot (rigid-body velocities and
    // contact-point positions/velocities) as computed by the cluster tree model versus the
    // rigid-body tree model.

    for (int k = 0; k < 20; k++)
    {
        // Forward kinematics
        this->initializeRandomStates();
        this->cluster_model.forwardKinematicsIncludingContactPoints();
        this->generic_model.forwardKinematicsIncludingContactPoints();
        this->rigid_body_model.forwardKinematicsIncludingContactPoints();

        // Verify link kinematics
        for (const auto &body : this->cluster_model.bodies())
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

        this->cluster_model.updateContactPointJacobians();
        this->rigid_body_model.updateContactPointJacobians();
        for (int j = 0; j < (int)this->cluster_model.contactPoints().size(); j++)
        {
            const ContactPoint<double> &cluster_cp = this->cluster_model.contactPoint(j);
            const ContactPoint<double> &rigid_body_cp = this->rigid_body_model.contactPoint(j);

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
            SVec<double> J_qdot = SVec<double>::Zero();
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

GTEST_TEST(ForwardKinematics, HumanoidModelComparison)
{
    // This test compares the forward kinematics of the MIT humanoid model with and without rotors 
    // to ensure that the only difference is the rotor joints.

    // Build the models
    MIT_Humanoid<double> robot_with_rotors;
    ClusterTreeModel<> rotor_model(robot_with_rotors.buildClusterTreeModel());

    MIT_Humanoid_no_rotors<double> robot_no_rotors;
    ClusterTreeModel<> no_rotor_model(robot_no_rotors.buildClusterTreeModel());

    for (int i = 0; i < 20; i++)
    {
        // Initialize Random States
        ModelState<> model_state;
        for (const auto &cluster : rotor_model.clusters())
        {
            JointState<> joint_state = cluster->joint_->randomJointState();
            model_state.push_back(joint_state);
        }
        rotor_model.setState(model_state);
        no_rotor_model.setState(model_state);

        // Forward Kinematics
        rotor_model.forwardKinematicsIncludingContactPoints();
        no_rotor_model.forwardKinematicsIncludingContactPoints();

        // Compare
        for (const auto &body : no_rotor_model.bodies())
        {
            const Vec3<double> p_rotor = rotor_model.getPosition(body.name_);
            const Vec3<double> p_no_rotor = no_rotor_model.getPosition(body.name_);
            GTEST_ASSERT_LT((p_rotor - p_no_rotor).norm(), tol);

            const Mat3<double> R_rotor = rotor_model.getOrientation(body.name_);
            const Mat3<double> R_no_rotor = no_rotor_model.getOrientation(body.name_);
            GTEST_ASSERT_LT((R_rotor - R_no_rotor).norm(), tol);

            const Vec3<double> v_rotor = rotor_model.getLinearVelocity(body.name_);
            const Vec3<double> v_no_rotor = no_rotor_model.getLinearVelocity(body.name_);
            GTEST_ASSERT_LT((v_rotor - v_no_rotor).norm(), tol);

            const Vec3<double> w_rotor = rotor_model.getAngularVelocity(body.name_);
            const Vec3<double> w_no_rotor = no_rotor_model.getAngularVelocity(body.name_);
            GTEST_ASSERT_LT((w_rotor - w_no_rotor).norm(), tol);
        }

        GTEST_ASSERT_EQ(rotor_model.contactPoints().size(), no_rotor_model.contactPoints().size());

        rotor_model.updateContactPointJacobians();
        no_rotor_model.updateContactPointJacobians();
        for (int j = 0; j < (int)rotor_model.contactPoints().size(); j++)
        {
            const ContactPoint<double> &rotor_cp = rotor_model.contactPoint(j);
            const ContactPoint<double> &no_rotor_cp = no_rotor_model.contactPoint(j);

            // Verify positions
            const Vec3<double> p_cp_rotor = rotor_cp.position_;
            const Vec3<double> p_cp_no_rotor = no_rotor_cp.position_;
            GTEST_ASSERT_LT((p_cp_rotor - p_cp_no_rotor).norm(), tol);

            // Verify velocities
            const Vec3<double> v_cp_rotor = rotor_cp.velocity_;
            const Vec3<double> v_cp_no_rotor = no_rotor_cp.velocity_;
            GTEST_ASSERT_LT((v_cp_rotor - v_cp_no_rotor).norm(), tol);

            // Verify jacobians
            const D6Mat<double> J_cp_rotor = rotor_cp.jacobian_;
            const D6Mat<double> J_cp_no_rotor = no_rotor_cp.jacobian_;
            GTEST_ASSERT_LT((J_cp_rotor - J_cp_no_rotor).norm(), tol);
        }
    }
}
