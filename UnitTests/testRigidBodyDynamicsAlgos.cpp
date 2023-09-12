#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "Dynamics/RigidBodyTreeModel.h"
#include "Robots/RobotTypes.h"

using namespace grbda;

static const double tol = 2e-8;

// The purpose of these tests is to ensure consistency between the outputs of the Rigid Body
// Dynamics Algorithms for our cluster tree model and the constrained rigid body tree
// (Featherstone) model

template <class T>
class RigidBodyDynamicsAlgosTest : public testing::Test
{
protected:
    RigidBodyDynamicsAlgosTest()
    {
        // TODO(@MatthewChignoli): In cases where we cannot build a random robot (e.g. Tello), we do not need to have multiple robots
        const int num_robots = 10;
        for (int i = 0; i < num_robots; i++)
        {
            T robot;
            ClusterTreeModel cluster_model(robot.buildClusterTreeModel());
            ClusterTreeModel generic_model = extractGenericJointModel(cluster_model);
            RigidBodyTreeModel lg_mult_custom_model(cluster_model,
                                                    FwdDynMethod::LagrangeMultiplierCustom);
            RigidBodyTreeModel lg_mult_eigen_model(cluster_model,
                                                   FwdDynMethod::LagrangeMultiplierEigen);
            RigidBodyTreeModel projection_model(cluster_model,
                                                FwdDynMethod::Projection);

            robots.push_back(robot);
            cluster_models.push_back(cluster_model);
            generic_models.push_back(generic_model);
            lg_mult_custom_models.push_back(lg_mult_custom_model);
            lg_mult_eigen_models.push_back(lg_mult_eigen_model);
            projection_models.push_back(projection_model);
        }
    }

    bool initializeRandomStates(const int robot_idx, bool use_spanning_state)
    {
        ModelState model_state;
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);

        for (const auto &cluster : cluster_models.at(robot_idx).clusters())
        {
            JointState joint_state = cluster->joint_->randomJointState();
            JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                   spanning_joint_state.position);
            spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                   spanning_joint_state.velocity);

            if (use_spanning_state)
                model_state.push_back(spanning_joint_state);
            else
                model_state.push_back(joint_state);
        }

        cluster_models[robot_idx].initializeState(model_state);
        generic_models[robot_idx].initializeState(model_state);
        lg_mult_custom_models[robot_idx].initializeState(spanning_joint_pos, spanning_joint_vel);
        lg_mult_eigen_models[robot_idx].initializeState(spanning_joint_pos, spanning_joint_vel);
        projection_models[robot_idx].initializeState(spanning_joint_pos, spanning_joint_vel);

        // Check for NaNs
        bool nan_detected = false;
        for (const auto &cluster : this->cluster_models[robot_idx].clusters())
        {
            if (cluster->joint_state_.position.hasNaN())
            {
                nan_detected = true;
                break;
            }
        }
        return nan_detected;
    }

    void setForcesForAllModels(std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs,
                               const int robot_idx)
    {
        cluster_models[robot_idx].initializeExternalForces(force_and_index_pairs);
        generic_models[robot_idx].initializeExternalForces(force_and_index_pairs);
        lg_mult_custom_models[robot_idx].initializeExternalForces(force_and_index_pairs);
        lg_mult_eigen_models[robot_idx].initializeExternalForces(force_and_index_pairs);
        projection_models[robot_idx].initializeExternalForces(force_and_index_pairs);
    }

    std::vector<T> robots;
    std::vector<ClusterTreeModel> cluster_models;
    std::vector<ClusterTreeModel> generic_models;
    std::vector<RigidBodyTreeModel> lg_mult_custom_models, lg_mult_eigen_models;
    std::vector<RigidBodyTreeModel> projection_models;
};

using testing::Types;

typedef Types<
    TeleopArm, Tello, TelloWithArms,
    MIT_Humanoid, MiniCheetah,
    RevoluteChainWithRotor<2>,
    RevoluteChainWithRotor<4>,
    RevoluteChainWithRotor<8>,
    RevolutePairChainWithRotor<2>,
    RevolutePairChainWithRotor<4>,
    RevolutePairChainWithRotor<8>,
    RevoluteTripleChainWithRotor<3>,
    RevoluteTripleChainWithRotor<6>,
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
        ClusterTreeModel &cluster_model = this->cluster_models.at(i);
        ClusterTreeModel &generic_model = this->generic_models.at(i);
        RigidBodyTreeModel &projection_model = this->projection_models.at(i);

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            const bool use_spanning_state = i % 2 == 0;
            bool nan_detected_in_state = this->initializeRandomStates(i, use_spanning_state);
            if (nan_detected_in_state)
            {
                j--;
                continue;
            }

            DMat<double> H_cluster = cluster_model.getMassMatrix();
            DMat<double> H_generic = generic_model.getMassMatrix();
            DMat<double> H_projection = projection_model.getMassMatrix();

            ASSERT_TRUE(isPositiveDefinite(H_cluster)) << H_cluster;
            GTEST_ASSERT_LT((H_cluster - H_generic).norm(), tol);
            GTEST_ASSERT_LT((H_cluster - H_projection).norm(), tol);
        }
    }
}

TYPED_TEST(RigidBodyDynamicsAlgosTest, BiasForceVector)
{
    // This test compares bias force vectors as computed by the cluster based model versus the
    // rigid body tree model, specifically via the projection method

    const int num_tests_per_robot = 20;
    for (int i = 0; i < (int)this->cluster_models.size(); i++)
    {
        ClusterTreeModel &cluster_model = this->cluster_models[i];
        ClusterTreeModel &generic_model = this->generic_models[i];
        RigidBodyTreeModel &projection_model = this->projection_models[i];

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            const bool use_spanning_state = i % 2 == 0;
            bool nan_detected_in_state = this->initializeRandomStates(i, use_spanning_state);
            if (nan_detected_in_state)
            {
                j--;
                continue;
            }

            DVec<double> C_cluster = cluster_model.getBiasForceVector();
            DVec<double> C_generic = generic_model.getBiasForceVector();
            DVec<double> C_projection = projection_model.getBiasForceVector();

            GTEST_ASSERT_LT((C_cluster - C_generic).norm(), tol);
            GTEST_ASSERT_LT((C_cluster - C_projection).norm(), tol);
        }
    }
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
        ClusterTreeModel &cluster_model = this->cluster_models[i];
        ClusterTreeModel &generic_model = this->generic_models[i];
        RigidBodyTreeModel &lg_mult_custom_model = this->lg_mult_custom_models[i];
        RigidBodyTreeModel &lg_mult_eigen_model = this->lg_mult_eigen_models[i];
        RigidBodyTreeModel &projection_model = this->projection_models[i];

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            // Set random state
            const bool use_spanning_state = i % 2 == 0;
            bool nan_detected_in_state = this->initializeRandomStates(i, use_spanning_state);
            if (nan_detected_in_state)
            {
                j--;
                continue;
            }

            // Set random spatial forces on bodies
            std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
            for (const auto &body : cluster_model.bodies())
                force_and_index_pairs.emplace_back(body.index_, SVec<double>::Random());
            this->setForcesForAllModels(force_and_index_pairs, i);

            // Forward Dynamics
            const DVec<double> tau = DVec<double>::Random(nv);

            const DVec<double> qdd_cluster = cluster_model.forwardDynamics(tau);
            const DVec<double> qdd_generic = generic_model.forwardDynamics(tau);
            const DVec<double> qdd_lg_custom_full = lg_mult_custom_model.forwardDynamics(tau);
            const DVec<double> qdd_lg_eigen_full = lg_mult_eigen_model.forwardDynamics(tau);
            const DVec<double> qdd_projection_full = projection_model.forwardDynamics(tau);

            // Convert between qdd and ydd
            const DVec<double> qdd_cluster_full = lg_mult_custom_model.yddToQdd(qdd_cluster);
            const DVec<double> qdd_lg_custom = lg_mult_custom_model.qddToYdd(qdd_lg_custom_full);
            const DVec<double> qdd_lg_eigen = lg_mult_eigen_model.qddToYdd(qdd_lg_eigen_full);
            const DVec<double> qdd_projection = projection_model.qddToYdd(qdd_projection_full);

            // Inverse Dynamics
            const DVec<double> tau_cluster = cluster_model.inverseDynamics(qdd_cluster);
            const DVec<double> tau_proj = projection_model.inverseDynamics(qdd_cluster);

            // Verify joint acceleration agreement
            GTEST_ASSERT_LT((qdd_cluster_full - qdd_lg_custom_full).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster_full - qdd_lg_eigen_full).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster_full - qdd_projection_full).norm(), tol);

            GTEST_ASSERT_LT((qdd_cluster - qdd_generic).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster - qdd_lg_custom).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster - qdd_lg_eigen).norm(), tol);
            GTEST_ASSERT_LT((qdd_cluster - qdd_projection).norm(), tol);

            // Verify joint torque agreement
            GTEST_ASSERT_LT((tau_cluster - tau).norm(), tol);
            GTEST_ASSERT_LT((tau_proj - tau).norm(), tol);
        }
    }
}

TYPED_TEST(RigidBodyDynamicsAlgosTest, LambdaInv)
{
    // This test validates that the generalized Extended-Force-Propagator Algorithm works as
    // expected by comparing it to the output of the projection model-based approach which computes
    // J * (G^T * H * G)^-1 * J^T

    const int num_tests_per_robot = 20;
    for (int i = 0; i < (int)this->cluster_models.size(); i++)
    {
        ClusterTreeModel &cluster_model = this->cluster_models[i];
        ClusterTreeModel &gen_model = this->generic_models[i];
        RigidBodyTreeModel &proj_model = this->projection_models[i];

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            const bool use_spanning_state = i % 2 == 0;
            bool nan_detected_in_state = this->initializeRandomStates(i, use_spanning_state);
            if (nan_detected_in_state)
            {
                j--;
                continue;
            }

            if (cluster_model.getNumEndEffectors() == 0)
            {
                std::cout << "No end effectors in cluster model. Skipping test." << std::endl;
                return;
            }

            const DMat<double> lambda_inv = cluster_model.inverseOperationalSpaceInertiaMatrix();
            const DMat<double> lambda_inv_gen = gen_model.inverseOperationalSpaceInertiaMatrix();
            const DMat<double> lambda_inv_proj = proj_model.inverseOperationalSpaceInertiaMatrix();

            GTEST_ASSERT_LT((lambda_inv - lambda_inv_gen).norm(), tol);
            GTEST_ASSERT_LT((lambda_inv - lambda_inv_proj).norm(), tol);
        }
    }
}

TYPED_TEST(RigidBodyDynamicsAlgosTest, ApplyTestForceTest)
{
    // This test validates that the generalized Apply Test Force Algorithm works as expected.
    // We compare the elements of the inverse operational space inertia matrix (lambda_inv) and the
    // resulting change in joint state (dstate) as computed by the Apply Test Force Algorithm
    // (which is based on the EFPA) to the same quantities computed by the classic J H^-1 J^T

    const int num_tests_per_robot = 20;
    for (int i = 0; i < (int)this->cluster_models.size(); i++)
    {
        ClusterTreeModel &cluster_model = this->cluster_models[i];
        RigidBodyTreeModel &projection_model = this->projection_models[i];

        const int nq = cluster_model.getNumPositions();
        const int nv = cluster_model.getNumDegreesOfFreedom();

        std::vector<Vec3<double>> test_forces;
        test_forces.push_back(Vec3<double>{1., 0., 0.});
        test_forces.push_back(Vec3<double>{0., 1., 0.});
        test_forces.push_back(Vec3<double>{0., 0., 1.});

        for (int j = 0; j < num_tests_per_robot; j++)
        {
            // Set random state
            const bool use_spanning_state = i % 2 == 0;
            bool nan_detected_in_state = this->initializeRandomStates(i, use_spanning_state);
            if (nan_detected_in_state)
            {
                j--;
                continue;
            }

            cluster_model.contactJacobians();
            for (const ContactPoint &cp : cluster_model.contactPoints())
            {
                const D3Mat<double> J = cluster_model.contactJacobian(cp.name_).bottomRows<3>();
                const DMat<double> H = cluster_model.getMassMatrix();
                const DMat<double> H_inv = H.inverse();
                const DMat<double> inv_ops_inertia = J * H_inv * J.transpose();

                for (const Vec3<double> &test_force : test_forces)
                {
                    const DVec<double> dstate_iosi = H_inv * (J.transpose() * test_force);
                    const double lambda_inv_iosi = test_force.dot(inv_ops_inertia * test_force);

                    DVec<double> dstate_efpa = DVec<double>::Zero(nv);
                    const double lambda_inv_efpa =
                        cluster_model.applyTestForce(cp.name_, test_force, dstate_efpa);

                    DVec<double> dstate_proj = DVec<double>::Zero(nv);
                    const double lambda_inv_proj =
                        projection_model.applyTestForce(cp.name_, test_force, dstate_proj);

                    GTEST_ASSERT_LT(std::fabs(lambda_inv_efpa - lambda_inv_iosi), tol);
                    GTEST_ASSERT_LT((dstate_efpa - dstate_iosi).norm(), tol);

                    GTEST_ASSERT_LT(std::fabs(lambda_inv_proj - lambda_inv_iosi), tol);
                    GTEST_ASSERT_LT((dstate_proj - dstate_iosi).norm(), tol);
                }
            }
        }
    }
}
