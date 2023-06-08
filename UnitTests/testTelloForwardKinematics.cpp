#include "gtest/gtest.h"

#include "DynamicsEngine/RigidBodyTreeModel.h"
#include "Robots/Tello.hpp"

static const double tol = 1e-5;

template <class T>
class TelloRigidBodyKinematicsTest : public testing::Test
{
protected:
    TelloRigidBodyKinematicsTest() : cluster_model(robot.buildClusterTreeModel()),
	                             rigid_body_model(cluster_model) {}

    T robot;
    ClusterTreeModel cluster_model;
    RigidBodyTreeModel rigid_body_model;
};

using testing::Types;

typedef Types<Tello> Robots;

TYPED_TEST_SUITE(TelloRigidBodyKinematicsTest, Robots);

TYPED_TEST(TelloRigidBodyKinematicsTest, ForwardKinematics)
{
    const int n_qd = 2; // 2 dependent joint config in Tello
    const int n_qd_dot = 2; // 2 dependent joint vel in Tello

    for (int k = 0; k < 20; k++)
    {

	// Set random state
        DVec<double> dependent_state = DVec<double>::Random(n_qd + n_qd_dot);
	Vec2<double> y = Vec2<double>::Zero(2);
	Vec2<double> y_dot = Vec2<double>::Zero(2);

	Vec2<double> qd = dependent_state.head(2);
	vector<double *> arg = {qd.data()};
	vector<double *> res = {y.data()};
	casadi_interface(arg, res, y.size(),
			IK_dependent_state_to_y,
			IK_dependent_state_to_y_sparsity_out,
			IK_dependent_state_to_y_work);

	if (isnan(y[0])) continue;
	if (isnan(y[1])) continue;
	Vec2<double> qd_dot = dependent_state.tail(2);
	arg = {qd.data(), qd_dot.data()};
	res = {y_dot.data()};
	casadi_interface(arg, res, y_dot.size(),
			IK_dependent_state_to_y_dot,
			IK_dependent_state_to_y_dot_sparsity_out,
			IK_dependent_state_to_y_dot_work); 
	DVec<double> state(8);
	state << y, qd, y_dot, qd_dot;
	this->cluster_model.initializeTelloIndependentStates(state.head(4), y_dot);
	this->rigid_body_model.initializeStates(state.head(4), state.tail(4));

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

TYPED_TEST(TelloRigidBodyKinematicsTest, MotionSubspaceApparentDerivative)
{
    // This test compares the analyticall computed apparent derivative of the motion subspace
    // (S_ring) to the derivative computed via finite difference

    double dt = 0.00001;
    const int n_qd = 2; // 2 dependent joint config in Tello
    const int n_qd_dot = 2; // 2 dependent joint vel in Tello

    for (int k = 0; k < 20; k++)
    {
	DVec<double> dependent_state = DVec<double>::Random(n_qd + n_qd_dot);
	Vec2<double> y = Vec2<double>::Zero(2);
	Vec2<double> y_dot = Vec2<double>::Zero(2);
	Vec2<double> qd = dependent_state.head(2);
	
	vector<double *> arg = {qd.data()};
	vector<double *> res = {y.data()};
	casadi_interface(arg, res, y.size(),
			IK_dependent_state_to_y,
			IK_dependent_state_to_y_sparsity_out,
			IK_dependent_state_to_y_work);

	if (isnan(y[0])) continue;
	if (isnan(y[1])) continue;
	Vec2<double> qd_dot = dependent_state.tail(2);
	arg = {qd.data(), qd_dot.data()};
	res = {y_dot.data()};
	casadi_interface(arg, res, y_dot.size(),
			IK_dependent_state_to_y_dot,
			IK_dependent_state_to_y_dot_sparsity_out,
			IK_dependent_state_to_y_dot_work); 
	DVec<double> state(8);
	state << y, qd, y_dot, qd_dot;
	Vec4<double> q_dot = state.tail(4);

	this->cluster_model.initializeTelloIndependentStates(state.head(4), y_dot);
        this->cluster_model.forwardKinematics();

        for (auto &cluster : this->cluster_model.clusters())
        {
            auto joint = cluster->joint_;

            DMat<double> S_ring = joint->S_ring();
            DMat<double> S = joint->S();

	    DVec<double> q_plus = cluster->q_ + dt * q_dot;
	    joint->updateKinematics(q_plus, cluster->qd_);

            DMat<double> S_plus = joint->S();
            DMat<double> S_ring_fd = (S_plus - S) / dt;

            GTEST_ASSERT_LT((S_ring - S_ring_fd).norm(), 0.008);
        }
    }
}
