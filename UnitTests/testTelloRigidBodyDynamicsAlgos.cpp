#include "gtest/gtest.h"

#include "Dynamics/RigidBodyTreeModel.h"
#include "Robots/Tello.hpp"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

static const double tol = 1e-5;

// The purpose of these tests is to ensure consistency between the outputs of the Rigid Body
// Dynamics Algorithms for our cluster tree model and the constrained rigid body tree
// (Featherstone) model

template <class T>
class TelloRigidBodyDynamicsAlgoTest : public testing::Test
{
protected:
    TelloRigidBodyDynamicsAlgoTest()
        : cluster_model(robot.buildClusterTreeModel()),
          lagrange_mult_model{cluster_model, ForwardDynamicsMethod::LagrangeMultiplier},
          projection_model{cluster_model, ForwardDynamicsMethod::Projection},
          t_cluster(0), t_lagrange(0), t_projection(0) 
    {}

    void setStateForAllModels(DVec<double> q, DVec<double> qd)
    {
	cluster_model.initializeTelloIndependentStates(q, qd.head(2));
	lagrange_mult_model.initializeStates(q, qd);
        projection_model.initializeStates(q, qd);
    }

    void setForcesForAllModels(std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs)
    {
        cluster_model.initializeExternalForces(force_and_index_pairs);
        lagrange_mult_model.initializeExternalForces(force_and_index_pairs);
        projection_model.initializeExternalForces(force_and_index_pairs);
    }

    void printAverageComputationTimes(double num_samples)
    {
        if (t_cluster > 0)
            std::cout << "Cluster   : " << t_cluster / num_samples << "ms\n";
        if (t_lagrange > 0)
            std::cout << "Lagrange  : " << t_lagrange / num_samples << "ms\n";
        if (t_projection > 0)
            std::cout << "Projection: " << t_projection / num_samples << "ms\n\n\n";
    }

    T robot;
    ClusterTreeModel cluster_model;
    RigidBodyTreeModel lagrange_mult_model, projection_model;

    Timer timer;
    double t_cluster, t_lagrange, t_projection;
};

using testing::Types;

typedef Types<Tello> Robots;

TYPED_TEST_SUITE(TelloRigidBodyDynamicsAlgoTest, Robots);

TYPED_TEST(TelloRigidBodyDynamicsAlgoTest, MassMatrix)
{
    // This test compares mass matrices as computed by the cluster based model versus the rigid body
    // tree model, specifically via the projection method

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
	const int n_qd = 2;
	const int n_qd_dot = 2;
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

	this->setStateForAllModels(state.head(4), state.tail(4));

        this->timer.start();
        DMat<double> H_cluster = this->cluster_model.getMassMatrix();
        this->t_cluster += this->timer.getMs();

	this->projection_model.extractLoopClosureFunctionsFromClusterModel(this->cluster_model);

        this->timer.start();
        DMat<double> H_projection = this->projection_model.getMassMatrix();
        this->t_projection += this->timer.getMs();

        GTEST_ASSERT_LT((H_cluster - H_projection).norm(), tol);
    }

    std::cout << "\n**Avergage Mass Matrix Computation Time**" << std::endl;
    this->printAverageComputationTimes(num_tests);
}

TYPED_TEST(TelloRigidBodyDynamicsAlgoTest, BiasForceVector)
{
    // This test compares bias force vectors as computed by the cluster based model versus the
    // rigid body tree model, specifically via the projection method

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
	const int n_qd = 2;
	const int n_qd_dot = 2;
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

	this->setStateForAllModels(state.head(4), state.tail(4));

        this->timer.start();
        DVec<double> C_cluster = this->cluster_model.getBiasForceVector();
        this->t_cluster += this->timer.getMs();

	this->projection_model.extractLoopClosureFunctionsFromClusterModel(this->cluster_model);

        this->timer.start();
	DMat<double> H_proj_del = this->projection_model.getMassMatrix();
        DVec<double> C_projection = this->projection_model.getBiasForceVector();
        this->t_projection += this->timer.getMs();

        GTEST_ASSERT_LT((C_cluster - C_projection).norm(), tol);
    }

    std::cout << "\n**Avergage Bias Force Vector Computation Time**" << std::endl;
    this->printAverageComputationTimes(num_tests);
}

TYPED_TEST(TelloRigidBodyDynamicsAlgoTest, ForwardAndInverseDyanmics)
{
    // This test compares forward dynamics as computed by the cluster based model to the forward
    // dynamics as computed by the rigid body tree model (via both the Lagrange multiplier method
    // and the projected equation of motion method). The inverse dynamics output is also validated
    // against the forward dynamics calculation.

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
        const int nv = this->cluster_model.getNumDegreesOfFreedom();
	const int n_qd = 2;
	const int n_qd_dot = 2;
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

        // Set random state
	this->setStateForAllModels(state.head(4), state.tail(4));

        // Set random spatial forces on bodies
        std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
        for (const auto &body : this->cluster_model.bodies())
            force_and_index_pairs.emplace_back(body.index_, SVec<double>::Random());
        this->setForcesForAllModels(force_and_index_pairs);

        // Forward Dynamics
        DVec<double> tau = DVec<double>::Random(nv);
        this->timer.start();
        DVec<double> qdd_cluster = this->cluster_model.forwardDynamics(tau);
        this->t_cluster += this->timer.getMs();
	this->lagrange_mult_model.extractLoopClosureFunctionsFromClusterModel(this->cluster_model);
	this->projection_model.extractLoopClosureFunctionsFromClusterModel(this->cluster_model);
        DVec<double> qdd_cluster_full = this->lagrange_mult_model.yddToQdd(qdd_cluster);

        this->timer.start();
        DVec<double> qdd_lagrange_full = this->lagrange_mult_model.forwardDynamics(tau);
        this->t_lagrange += this->timer.getMs();
        DVec<double> qdd_lagrange = this->lagrange_mult_model.qddToYdd(qdd_lagrange_full);

        this->timer.start();
        DVec<double> qdd_projection_full = this->projection_model.forwardDynamics(tau);
        this->t_projection += this->timer.getMs();
        DVec<double> qdd_projection = this->projection_model.qddToYdd(qdd_projection_full);

        // Inverse Dynamics
        DVec<double> tau_cluster = this->cluster_model.inverseDyamics(qdd_cluster);

        // Verify joint acceleration agreement
        GTEST_ASSERT_LT((qdd_cluster_full - qdd_lagrange_full).norm(), tol);
        GTEST_ASSERT_LT((qdd_cluster_full - qdd_projection_full).norm(), tol);
	
        GTEST_ASSERT_LT((qdd_cluster - qdd_lagrange).norm(), tol);
        GTEST_ASSERT_LT((qdd_cluster - qdd_projection).norm(), tol);

        // Verify joint torque agreement
        GTEST_ASSERT_LT((tau_cluster - tau).norm(), tol);
    }

    std::cout << "\n**Avergage Forward Dynamics Computation Time**" << std::endl;
    this->printAverageComputationTimes(num_tests);

}

// GTEST_TEST(OpenChain2DoF, ApplyTestForceTest)
// {
//     Openchain2DoF model_lagrange{};
//     ClusterTreeModel model_featherstone = model_lagrange.buildClusterTreeModel();

//     // Set random state
//     DVec<double> state = Vec4<double>::Random();
//     model_lagrange.setState(state);
//     model_featherstone.initializeIndependentStates(state.head<2>(), state.tail<2>());

//     model_lagrange._UpdateDynamics_ABA(Vec2<double>::Zero());
//     Mat3<double> inv_ops_inertia = model_lagrange.inverse_operational_space_inertia_matrix();

//     Vec3<double> test_force = Vec3<double>{1., 0., 0.};
//     DVec<double> dstate_out;

//     double lambda_inv_x =
//         model_featherstone.applyLocalFrameTestForceAtConactPoint(Vec3<double>{1., 0., 0.},
//                                                                  "tip-of-link-2",
//                                                                  dstate_out);
//     GTEST_ASSERT_LT(fabs(inv_ops_inertia(0, 0) - lambda_inv_x), tol);

//     double lambda_inv_y =
//         model_featherstone.applyLocalFrameTestForceAtConactPoint(Vec3<double>{0., 1., 0.},
//                                                                  "tip-of-link-2",
//                                                                  dstate_out);
//     GTEST_ASSERT_LT(fabs(inv_ops_inertia(1, 1) - lambda_inv_y), tol);

//     double lambda_inv_z =
//         model_featherstone.applyLocalFrameTestForceAtConactPoint(Vec3<double>{0., 0., 1.},
//                                                                  "tip-of-link-2",
//                                                                  dstate_out);
//     GTEST_ASSERT_LT(fabs(inv_ops_inertia(2, 2) - lambda_inv_z), tol);
// }
