#include "gtest/gtest.h"

#include "DynamicsEngine/RigidBodyTreeModel.h"
#include "DynamicsEngine/ReflectedInertiaTreeModel.h"
#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

static const double tol = 1e-5;

template <class T>
class ReflectedInertiaDynamicsAlgosTest : public testing::Test
{
protected:
    ReflectedInertiaDynamicsAlgosTest()
        : cluster_model(robot.buildClusterTreeModel()),
          reflected_inertia_model{cluster_model} {}

    void setStateForAllModels(DVec<double> q, DVec<double> qd)
    {
        cluster_model.initializeIndependentStates(q, qd);
        reflected_inertia_model.initializeIndependentStates(q, qd);
    }

    void setForcesForAllModels(std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs)
    {
        cluster_model.initializeExternalForces(force_and_index_pairs);
        reflected_inertia_model.initializeExternalForces(force_and_index_pairs);
    }

    T robot = T(false);
    ClusterTreeModel cluster_model;
    ReflectedInertiaTreeModel reflected_inertia_model;
};

using testing::Types;

typedef Types<RevoluteChainWithRotor<2>,
              RevoluteChainWithRotor<4>,
              RevolutePairChainWithRotor<2>,
              RevolutePairChainWithRotor<4>>
    RobotsCompatibleWithReflectedInertiaModel;

TYPED_TEST_SUITE(ReflectedInertiaDynamicsAlgosTest, RobotsCompatibleWithReflectedInertiaModel);

TYPED_TEST(ReflectedInertiaDynamicsAlgosTest, ForwardKinematics)
{
    // This test compares the forward kinematics of a robot (rigid-body velocities and 
    // contact-point positions/velocities) as computed by the cluster tree model versus the 
    // reflected inertia tree model.

    const int nq = this->cluster_model.getNumPositions();
    const int nv = this->cluster_model.getNumDegreesOfFreedom();

    for (int k = 0; k < 20; k++)
    {

        // Set random state
        DVec<double> state = DVec<double>::Random(nq + nv);
        this->setStateForAllModels(state.head(nq), state.tail(nv));
        
        // Forward kinematics
        this->cluster_model.forwardKinematics();
        this->reflected_inertia_model.forwardKinematics();

        // Verify spatial velocity agreement of bodies
        DVec<double> v_cluster = DVec<double>::Zero(6 * this->cluster_model.getNumBodies());
        int i = 0;
        for (const auto &cluster : this->cluster_model.clusters())
        {
            v_cluster.segment(i, cluster->motion_subspace_dimension_) = cluster->v_;
            i += cluster->motion_subspace_dimension_;
        }

        DVec<double> v_links = DVec<double>::Zero(6 * this->reflected_inertia_model.getNumBodies());
        i = 0;
        for (const auto &node : this->reflected_inertia_model.nodes())
        {
            v_links.segment<6>(i) = node->v_;
            i += 6;
        }

        const DVec<int> independent_coord_indices =
            this->reflected_inertia_model.getIndependentCoordinateIndices();
        for (int j = 0; j < this->reflected_inertia_model.getNumBodies(); j++)
        {
            const auto &v1 = v_cluster.segment<6>(6 * independent_coord_indices[j]);
            const auto &v2 = v_links.segment<6>(6 * j);
            GTEST_ASSERT_LT((v1 - v2).norm(), tol);
        }

        // Verify cartesian velocity of contact points
        GTEST_ASSERT_EQ(this->cluster_model.contactPoints().size(),
                        this->reflected_inertia_model.contactPoints().size());

        for (int j = 0; j < (int)this->cluster_model.contactPoints().size(); j++)
        {
            // Verify positions
            Vec3<double> p_cp_cluster = this->cluster_model.contactPoint(j).position_;
            Vec3<double> p_cp_ref_inertia = this->reflected_inertia_model.contactPoint(j).position_;
            GTEST_ASSERT_LT((p_cp_cluster - p_cp_ref_inertia).norm(), tol);

            // Verify velocities
            Vec3<double> v_cp_cluster = this->cluster_model.contactPoint(j).velocity_;
            Vec3<double> v_cp_ref_inertia = this->reflected_inertia_model.contactPoint(j).velocity_;
            GTEST_ASSERT_LT((v_cp_cluster - v_cp_ref_inertia).norm(), tol);
        }
    }
}

TYPED_TEST(ReflectedInertiaDynamicsAlgosTest, ComparisonAgainstLagrangianDerivation)
{
    // This test compares equations of motion for 3 cases:
    // 1) Cluster based model versus the exact Lagrangian derivation (via MATLAB CasADi codegen)
    // 2) Standard ABA with refelected inertia vs Lagrangian derivation that encodes the same
    // approximation
    // 3) Projection model with full block diagonal reflected inertia vs. Lagrangian derivation
    // that encodes the same approximation

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
        const int nq = this->cluster_model.getNumPositions();
        const int nv = this->cluster_model.getNumDegreesOfFreedom();

        DVec<double> state = DVec<double>::Random(nq + nv);
        const DVec<double> q = state.head(nq);
        const DVec<double> qd = state.tail(nv);
        this->setStateForAllModels(q, qd);
        DVec<double> tau = DVec<double>::Random(nv);

        // TODO(@MatthewChignoli): Apply spatial forces to bodies for the lagrangian derivation?
        // Set random spatial forces on bodies
        // std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs;
        // const DVec<int> &ind_coord_indices =
        //     this->reflected_inertia_model.getIndependentCoordinateIndices();
        // for (int j = 0; j < ind_coord_indices.rows(); j++)
        //     force_and_index_pairs.emplace_back(ind_coord_indices[j], SVec<double>::Random());
        // this->setForcesForAllModels(force_and_index_pairs);

        // Case 1: Cluster based model versus the exact Lagrangian derivation
        const DVec<double> qdd1 = this->cluster_model.forwardDynamics(tau);
        const DVec<double> qdd2 = this->robot.forwardDynamics(q, qd, tau);
        GTEST_ASSERT_LT((qdd1 - qdd2).norm(), tol);

        // Case 2: Standard ABA with reflected inertia vs Lagrangian derivation that encodes the
        // same approximation
        const DVec<double> qdd_rfd1 = this->reflected_inertia_model.forwardDynamics(tau);
        const DVec<double> qdd_rfd2 = this->robot.forwardDynamicsReflectedInertiaDiag(q, qd, tau);
        GTEST_ASSERT_LT((qdd_rfd1 - qdd_rfd2).norm(), tol);

        // TODO(@MatthewChignoli): Replace with the cluster model ref inertia ABA
        // Case 3: Projection model with full block diagonal reflected inertia vs. Lagrangian
        // derivation that encodes the same approximation
        const DVec<double> qdd_rf1 = this->reflected_inertia_model.forwardDynamicsHandC(tau);
        const DVec<double> qdd_rf2 = this->robot.forwardDynamicsReflectedInertia(q, qd, tau);
        GTEST_ASSERT_LT((qdd_rf1 - qdd_rf2).norm(), tol);
    }
}
