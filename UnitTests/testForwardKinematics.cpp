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

    const int nq = this->cluster_model.getNumPositions();
    const int nv = this->cluster_model.getNumDegreesOfFreedom();

    for (int k = 0; k < 20; k++)
    {

        // Set random state
        DVec<double> state = DVec<double>::Random(nq + nv);
        SpanningState<double> joint_pos(DVec<double>::Random(nq));
        IndependentState<double> joint_vel(DVec<double>::Random(nv));
        this->cluster_model.initializeIndependentStates(state.head(nq), state.tail(nv));
        this->rigid_body_model.initializeIndependentStates(state.head(nq), state.tail(nv));

        // this->cluster_model.initializeState(joint_pos, joint_vel);
        if (joint_pos.isSpanning())
        {
            std::cout << "Joint pos is spanning" << std::endl;
        }

        State<double> joint_pos2 = joint_pos;
        if (joint_pos2.isSpanning())
        {
            std::cout << "Joint pos is spanning" << std::endl;
        }

        // Forward kinematics
        this->cluster_model.forwardKinematics();
        this->rigid_body_model.forwardKinematics();

        // Verify spatial velocity agreement of bodies
        DVec<double> v_cluster = DVec<double>::Zero(6 * this->cluster_model.getNumBodies());
        int i = 0;
        for (const auto &cluster : this->cluster_model.clusters())
        {
            v_cluster.segment(i, cluster->motion_subspace_dimension_) = cluster->v_;
            i += cluster->motion_subspace_dimension_;
        }

        DVec<double> v_rigid_body = DVec<double>::Zero(6 * this->rigid_body_model.getNumBodies());
        i = 0;
        for (const auto &node : this->rigid_body_model.nodes())
        {
            v_rigid_body.segment<6>(i) = node->v_;
            i += 6;
        }

        GTEST_ASSERT_EQ(v_cluster.rows(), v_rigid_body.rows());
        GTEST_ASSERT_LT((v_cluster - v_rigid_body).norm(), tol);

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

    for (int k = 0; k < 20; k++)
    {
        DVec<double> y = DVec<double>::Random(nq);
        DVec<double> yd = DVec<double>::Random(nv);
        this->cluster_model.initializeIndependentStates(y, yd);
        this->cluster_model.forwardKinematics();

        for (auto &cluster : this->cluster_model.clusters())
        {
            auto joint = cluster->joint_;

            DMat<double> S_ring = joint->S_ring();
            // DMat<double> S = joint->S();

            // TODO(@MatthewChignoli): For now we are assuming independent, but that's wrong
            // IndependentState<double> q_plus = cluster->q_ + dt * cluster->qd_;
            IndependentState<double> q_plus(cluster->q_ + dt * cluster->qd_);
            joint->updateKinematics(q_plus, cluster->qd_);
            DMat<double> S_plus = joint->S();

            // IndependentState<double> q_minus = cluster->q_ - dt * cluster->qd_;
            IndependentState<double> q_minus(cluster->q_ - dt * cluster->qd_);
            joint->updateKinematics(q_minus, cluster->qd_);
            DMat<double> S_minus = joint->S();

            DMat<double> S_ring_fd = (S_plus - S_minus) / (2 * dt);

            GTEST_ASSERT_LT((S_ring - S_ring_fd).norm(), tol);
        }
    }
}
