#include "gtest/gtest.h"

#include "Dynamics/RigidBodyTreeModel.h"
#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

static const double tol = 1e-5;

// The purpose of these tests is to ensure consistency between the outputs of the Rigid Body
// Dynamics Algorithms for our cluster tree model and the constrained rigid body tree
// (Featherstone) model

template <class T>
class RigidBodyDynamicsAlgosTest : public testing::Test
{
protected:
    RigidBodyDynamicsAlgosTest() : t_cluster(0), t_lagrange(0), t_projection(0)
    {
        const int num_robots = 10;
        for (int i = 0; i < num_robots; i++)
        {
            T robot;
            ClusterTreeModel cluster_model(robot.buildClusterTreeModel());
            RigidBodyTreeModel lagrange_mult_model(cluster_model,
                                                   ForwardDynamicsMethod::LagrangeMultiplier);
            RigidBodyTreeModel projection_model(cluster_model,
                                                ForwardDynamicsMethod::Projection);

            robots.push_back(robot);
            cluster_models.push_back(cluster_model);
            lagrange_mult_models.push_back(lagrange_mult_model);
            projection_models.push_back(projection_model);
        }
    }

    void setStateForAllModels(DVec<double> q, DVec<double> qd, const int robot_idx)
    {
        cluster_models[robot_idx].initializeIndependentStates(q, qd);
        lagrange_mult_models[robot_idx].initializeIndependentStates(q, qd);
        projection_models[robot_idx].initializeIndependentStates(q, qd);
    }

    void setForcesForAllModels(std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs,
                               const int robot_idx)
    {
        cluster_models[robot_idx].initializeExternalForces(force_and_index_pairs);
        lagrange_mult_models[robot_idx].initializeExternalForces(force_and_index_pairs);
        projection_models[robot_idx].initializeExternalForces(force_and_index_pairs);
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

    std::vector<T> robots;
    std::vector<ClusterTreeModel> cluster_models;
    std::vector<RigidBodyTreeModel> lagrange_mult_models, projection_models;

    Timer timer;
    double t_cluster, t_lagrange, t_projection;
};

using testing::Types;

typedef Types<
    TeleopArm,
    RevoluteChainWithRotor<2>,
    RevoluteChainWithRotor<4>,
    RevoluteChainWithRotor<8>,
    RevolutePairChainWithRotor<2>,
    RevolutePairChainWithRotor<4>,
    RevolutePairChainWithRotor<8>,
    RevoluteChainMultipleRotorsPerLink<2, 2>,
    RevoluteChainMultipleRotorsPerLink<4, 1>,
    RevoluteChainMultipleRotorsPerLink<4, 3>,
    RevoluteChainWithAndWithoutRotor<0ul, 8ul>,
    RevoluteChainWithAndWithoutRotor<4ul, 4ul>,
    RevoluteChainWithAndWithoutRotor<8ul, 0ul>>
    Robots;

TYPED_TEST_SUITE(RigidBodyDynamicsAlgosTest, Robots);

TYPED_TEST(RigidBodyDynamicsAlgosTest, MassMatrix)
{
    // This test compares mass matrices as computed by the cluster based model versus the rigid body
    // tree model, specifically via the projection method
    
    const int num_tests_per_robot = 20;
    for (int i = 0; i < (int)this->cluster_models.size(); i++)
    {
        ClusterTreeModel cluster_model = this->cluster_models[i];
        RigidBodyTreeModel projection_model = this->projection_models[i];

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            DVec<double> state = DVec<double>::Random(nq + nv);
            this->setStateForAllModels(state.head(nq), state.tail(nv), i);

            this->timer.start();
            DMat<double> H_cluster = cluster_model.getMassMatrix();
            this->t_cluster += this->timer.getMs();

            this->timer.start();
            DMat<double> H_projection = projection_model.getMassMatrix();
            this->t_projection += this->timer.getMs();

            ASSERT_TRUE(isPositiveDefinite(H_cluster)) << H_cluster;
            GTEST_ASSERT_LT((H_cluster - H_projection).norm(), tol);
        }
    }

    std::cout << "\n**Avergage Mass Matrix Computation Time**" << std::endl;
    const int num_tests = this->cluster_models.size() * num_tests_per_robot;
    this->printAverageComputationTimes(num_tests);
}

TYPED_TEST(RigidBodyDynamicsAlgosTest, BiasForceVector)
{
    // This test compares bias force vectors as computed by the cluster based model versus the
    // rigid body tree model, specifically via the projection method

    const int num_tests_per_robot = 20;
    for (int i = 0; i < (int)this->cluster_models.size(); i++)
    {
        ClusterTreeModel cluster_model = this->cluster_models[i];
        RigidBodyTreeModel projection_model = this->projection_models[i];

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            DVec<double> state = DVec<double>::Random(nq + nv);
            this->setStateForAllModels(state.head(nq), state.tail(nv), i);

            this->timer.start();
            DVec<double> C_cluster = cluster_model.getBiasForceVector();
            this->t_cluster += this->timer.getMs();

            this->timer.start();
            DVec<double> C_projection = projection_model.getBiasForceVector();
            this->t_projection += this->timer.getMs();

            GTEST_ASSERT_LT((C_cluster - C_projection).norm(), tol);
        }
    }

    std::cout << "\n**Avergage Bias Force Vector Computation Time**" << std::endl;
    const int num_tests = this->cluster_models.size() * num_tests_per_robot;
    this->printAverageComputationTimes(num_tests);
}

TYPED_TEST(RigidBodyDynamicsAlgosTest, ForwardAndInverseDyanmics)
{
    // This test compares forward dynamics as computed by the cluster based model to the forward
    // dynamics as computed by the rigid body tree model (via both the Lagrange multiplier method
    // and the projected equation of motion method). The inverse dynamics output is also validated
    // against the forward dynamics calculation.

    const int num_tests_per_robot = 20;
    for (int i = 0; i < (int)this->cluster_models.size(); i++)
    {
        ClusterTreeModel cluster_model = this->cluster_models[i];
        RigidBodyTreeModel lagrange_mult_model = this->lagrange_mult_models[i];
        RigidBodyTreeModel projection_model = this->projection_models[i];

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            // Set random state
            DVec<double> state = DVec<double>::Random(nq + nv);
            this->setStateForAllModels(state.head(nq), state.tail(nv), i);

            // Set random spatial forces on bodies
            std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
            for (const auto &body : cluster_model.bodies())
                force_and_index_pairs.emplace_back(body.index_, SVec<double>::Random());
            this->setForcesForAllModels(force_and_index_pairs, i);

            // Forward Dynamics
            DVec<double> tau = DVec<double>::Random(nv);

            this->timer.start();
            DVec<double> qdd_cluster = cluster_model.forwardDynamics(tau);
            this->t_cluster += this->timer.getMs();
            DVec<double> qdd_cluster_full = lagrange_mult_model.yddToQdd(qdd_cluster);

            this->timer.start();
            DVec<double> qdd_lagrange_full = lagrange_mult_model.forwardDynamics(tau);
            this->t_lagrange += this->timer.getMs();
            DVec<double> qdd_lagrange = lagrange_mult_model.qddToYdd(qdd_lagrange_full);

            this->timer.start();
            DVec<double> qdd_projection_full = projection_model.forwardDynamics(tau);
            this->t_projection += this->timer.getMs();
            DVec<double> qdd_projection = projection_model.qddToYdd(qdd_projection_full);

            // Inverse Dynamics
            DVec<double> tau_cluster = cluster_model.inverseDyamics(qdd_cluster);

            // Verify joint acceleration agreement
            GTEST_ASSERT_LT((qdd_cluster_full - qdd_lagrange_full).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster_full - qdd_projection_full).norm(), tol);

            GTEST_ASSERT_LT((qdd_cluster - qdd_lagrange).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster - qdd_projection).norm(), tol);

            // Verify joint torque agreement
            GTEST_ASSERT_LT((tau_cluster - tau).norm(), tol);
        }
    }

    std::cout << "\n**Avergage Forward Dynamics Computation Time**" << std::endl;
    const int num_tests = this->cluster_models.size() * num_tests_per_robot;
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
