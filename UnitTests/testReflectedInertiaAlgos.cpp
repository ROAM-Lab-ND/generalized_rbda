#include "gtest/gtest.h"

#include "Dynamics/RigidBodyTreeModel.h"
#include "Dynamics/ReflectedInertiaTreeModel.h"
#include "Robots/RobotTypes.h"
#include "Utils/Utilities/Timer.h"

using namespace grbda;

static const double tol = 1e-5;

template <class T>
class ReflectedInertiaDynamicsAlgosTest : public testing::Test
{
protected:
    ReflectedInertiaDynamicsAlgosTest()
        : cluster_model(robot.buildClusterTreeModel()),
          reflected_inertia_model{cluster_model, RotorInertiaApproximation::BLOCK_DIAGONAL},
          reflected_inertia_diag_model{cluster_model, RotorInertiaApproximation::DIAGONAL} {}

    bool initializeRandomStates()
    {
        ModelState model_state;
        independent_joint_pos_ = DVec<double>::Zero(0);
        independent_joint_vel_ = DVec<double>::Zero(0);
        for (const auto &cluster : cluster_model.clusters())
        {
            JointState joint_state = cluster->joint_->randomJointState();

            if (joint_state.position.isSpanning() || joint_state.velocity.isSpanning())
                throw std::runtime_error("Initializing reflected inertia model requires all independent coordinates");

            independent_joint_pos_ = appendEigenVector(independent_joint_pos_,
                                                       joint_state.position);
            independent_joint_vel_ = appendEigenVector(independent_joint_vel_,
                                                       joint_state.velocity);

            model_state.push_back(joint_state);
        }

        cluster_model.initializeState(model_state);
        reflected_inertia_model.initializeIndependentStates(independent_joint_pos_,
                                                            independent_joint_vel_);
        reflected_inertia_diag_model.initializeIndependentStates(independent_joint_pos_,
                                                                 independent_joint_vel_);

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

    void setForcesForAllModels(std::vector<ExternalForceAndBodyIndexPair> force_and_index_pairs)
    {
        cluster_model.initializeExternalForces(force_and_index_pairs);
        reflected_inertia_model.initializeExternalForces(force_and_index_pairs);
        reflected_inertia_diag_model.initializeExternalForces(force_and_index_pairs);
    }

    T robot = T(false);
    ClusterTreeModel cluster_model;
    ReflectedInertiaTreeModel reflected_inertia_model;
    ReflectedInertiaTreeModel reflected_inertia_diag_model;

    DVec<double> independent_joint_pos_;
    DVec<double> independent_joint_vel_;
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

    const int nv = this->cluster_model.getNumDegreesOfFreedom();

    for (int k = 0; k < 20; k++)
    {

        // Set random state
        bool nan_detected_in_state = this->initializeRandomStates();
        if (nan_detected_in_state)
        {
            k--;
            continue;
        }

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

        this->cluster_model.contactJacobians();
        this->reflected_inertia_model.contactJacobians();
        for (int j = 0; j < (int)this->cluster_model.contactPoints().size(); j++)
        {
            const ContactPoint &cluster_cp = this->cluster_model.contactPoint(j);
            const ContactPoint &ref_inertia_cp = this->reflected_inertia_model.contactPoint(j);

            // Verify positions
            const Vec3<double> p_cp_cluster = cluster_cp.position_;
            const Vec3<double> p_cp_ref_inertia = ref_inertia_cp.position_;
            GTEST_ASSERT_LT((p_cp_cluster - p_cp_ref_inertia).norm(), tol);

            // Verify velocities
            const Vec3<double> v_cp_cluster = cluster_cp.velocity_;
            const Vec3<double> v_cp_ref_inertia = ref_inertia_cp.velocity_;
            GTEST_ASSERT_LT((v_cp_cluster - v_cp_ref_inertia).norm(), tol);

            // Verify jacobians
            const D6Mat<double> J_cp_cluster = cluster_cp.jacobian_;
            const D6Mat<double> J_cp_ref_inertia = ref_inertia_cp.jacobian_;
            GTEST_ASSERT_LT((J_cp_cluster - J_cp_ref_inertia).norm(), tol);
        }
    }
}

TYPED_TEST(ReflectedInertiaDynamicsAlgosTest, CompareFwdDynAgainstLagrangianDerivation)
{
    // This test compares the forward dynamics for 3 cases:
    // 1) Cluster based model versus the exact Lagrangian derivation (via MATLAB CasADi codegen)
    // 2) Standard ABA with diagonal refelected inertia vs Lagrangian derivation that encodes the
    // same approximation
    // 3) Reflected inertia model with full block diagonal reflected inertia vs. Lagrangian
    // derivation that encodes the same approximation

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
        const int nv = this->cluster_model.getNumDegreesOfFreedom();

        bool nan_detected_in_state = this->initializeRandomStates();
        if (nan_detected_in_state)
        {
            i--;
            continue;
        }

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
        const DVec<double> qdd2 = this->robot.forwardDynamics(this->independent_joint_pos_,
                                                              this->independent_joint_vel_, tau);
        GTEST_ASSERT_LT((qdd1 - qdd2).norm(), tol);

        // Case 2: Standard ABA with reflected inertia vs Lagrangian derivation that encodes the
        // same approximation
        const DVec<double> qdd_rfd1 = this->reflected_inertia_diag_model.forwardDynamics(tau);
        const DVec<double> qdd_rfd2 =
            this->robot.forwardDynamicsReflectedInertiaDiag(this->independent_joint_pos_,
                                                            this->independent_joint_vel_, tau);
        GTEST_ASSERT_LT((qdd_rfd1 - qdd_rfd2).norm(), tol);

        // Case 3: Full block diagonal reflected inertia vs. Lagrangian derivation that encodes the
        // same approximation
        const DVec<double> qdd_rf1 = this->reflected_inertia_model.forwardDynamics(tau);
        const DVec<double> qdd_rf2 =
            this->robot.forwardDynamicsReflectedInertia(this->independent_joint_pos_,
                                                        this->independent_joint_vel_, tau);
        GTEST_ASSERT_LT((qdd_rf1 - qdd_rf2).norm(), tol);
    }
}

TYPED_TEST(ReflectedInertiaDynamicsAlgosTest, CompareInvDynAgainstLagrangianDerivation)
{
    // This test compares the inverse dynamics for 3 cases:
    // 1) Cluster based model versus the exact Lagrangian derivation (via MATLAB CasADi codegen)
    // 2) Standard ABA with diagonal refelected inertia vs Lagrangian derivation that encodes the
    // same approximation
    // 3) Reflected inertia model with full block diagonal reflected inertia vs. Lagrangian
    // derivation that encodes the same approximation

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
        const int nv = this->cluster_model.getNumDegreesOfFreedom();

        bool nan_detected_in_state = this->initializeRandomStates();
        if (nan_detected_in_state)
        {
            i--;
            continue;
        }

        const DVec<double> ydd = DVec<double>::Random(nv);

        // Case 1: Cluster based model versus the exact Lagrangian derivation
        const DVec<double> tau1 = this->cluster_model.inverseDynamics(ydd);
        const DVec<double> tau2 = this->robot.inverseDynamics(this->independent_joint_pos_,
                                                              this->independent_joint_vel_, ydd);
        GTEST_ASSERT_LT((tau1 - tau2).norm(), tol);

        // Case 2: Standard ABA with reflected inertia vs Lagrangian derivation that encodes the
        // same approximation
        const DVec<double> tau_rfd1 = this->reflected_inertia_diag_model.inverseDynamics(ydd);
        const DVec<double> tau_rfd2 =
            this->robot.inverseDynamicsReflectedInertiaDiag(this->independent_joint_pos_,
                                                            this->independent_joint_vel_, ydd);
        GTEST_ASSERT_LT((tau_rfd1 - tau_rfd2).norm(), tol);

        // Case 3: Full block diagonal reflected inertia vs. Lagrangian derivation that encodes the
        // same approximation
        const DVec<double> tau_rf1 = this->reflected_inertia_model.inverseDynamics(ydd);
        const DVec<double> tau_rf2 =
            this->robot.inverseDynamicsReflectedInertia(this->independent_joint_pos_,
                                                        this->independent_joint_vel_, ydd);
        GTEST_ASSERT_LT((tau_rf1 - tau_rf2).norm(), tol);
    }
}

TYPED_TEST(ReflectedInertiaDynamicsAlgosTest, LambdaInv)
{
    // This test compares the computation of the inverse operational space inertia matrix for both
    // reflected inertia models using (1) the Extended-Force-Propagator Algorithm and (2) the
    // traditional Jacobian & Mass Matrix method

    const int num_tests = 100;
    for (int i = 0; i < num_tests; i++)
    {
        const int nv = this->cluster_model.getNumDegreesOfFreedom();

        bool nan_detected_in_state = this->initializeRandomStates();
        if (nan_detected_in_state)
        {
            i--;
            continue;
        }

        std::vector<ReflectedInertiaTreeModel> models{this->reflected_inertia_model,
                                                      this->reflected_inertia_diag_model};

        for (auto &model : models)
        {
            const DMat<double> lambda_inv = model.inverseOperationalSpaceInertiaMatrix();

            const DMat<double> H = model.getMassMatrix();
            DMat<double> J_stacked = DMat<double>::Zero(6 * model.getNumEndEffectors(),
                                                        model.getNumDegreesOfFreedom());
            int ee_cnt = 0;
            for (int k = 0; k < (int)model.contactPoints().size(); k++)
            {
                const ContactPoint &cp = model.contactPoint(k);
                if (!cp.is_end_effector_)
                    continue;
                J_stacked.middleRows<6>(6 * ee_cnt++) = model.bodyJacobian(cp.name_);
            }
            DMat<double> J_Hinv_JT = J_stacked * H.inverse() * J_stacked.transpose();

            GTEST_ASSERT_LT((lambda_inv - J_Hinv_JT).norm(), tol);
        }
    }
}
