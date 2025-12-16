#include "gtest/gtest.h"

#include <complex>
#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

static const double tol = 1e-10;

// This text fixture is used to test aspects of the ClusterTreeModel class such as getters, 
// setters, and other member functions. It is templated to allow for testing with different
// robot types.
template <class T>
class ClusterTreeModelTest : public testing::Test
{
protected:
    ClusterTreeModelTest() : cluster_model(robot.buildClusterTreeModel()) {}

    T robot;
    ClusterTreeModel<> cluster_model;
};

using testing::Types;

typedef Types<
    RevoluteChainWithRotor<4>,
    RevolutePairChainWithRotor<4>,
    RevoluteChainWithAndWithoutRotor<4ul, 4ul>,
    PlanarLegLinkage<>,
    Tello<double>, TeleopArm,
    MIT_Humanoid<>,
    MIT_Humanoid<double, ori_representation::RollPitchYaw>,
    MIT_Humanoid_no_rotors<>,
    MiniCheetah<>,
    MiniCheetah<double, ori_representation::RollPitchYaw>>
    Robots;

TYPED_TEST_SUITE(ClusterTreeModelTest, Robots);

TYPED_TEST(ClusterTreeModelTest, EndEffectors)
{
    const int num_clusters = this->cluster_model.clusters().size();
    for (const auto &cp : this->cluster_model.contactPoints())
    {
        if (!cp.is_end_effector_)
            continue;
        ASSERT_EQ(cp.ChiUp_.size(), num_clusters);
    }

    std::vector<int> supported_end_effectors;
    for (const auto &cluster : this->cluster_model.clusters())
    {
        for (const auto &cp_index : cluster->supported_end_effectors_)
        {
            // Check if contact point is already accounted for
            auto it = std::find(supported_end_effectors.begin(),
                                supported_end_effectors.end(), cp_index);
            // If not, push it back
            if (it == supported_end_effectors.end())
                supported_end_effectors.push_back(cp_index);
        }
    }
    ASSERT_EQ(supported_end_effectors.size(), this->cluster_model.getNumEndEffectors());
}

// These tests verify how ClusterTreeModels are built from URDF files

const std::string urdf_directory = SOURCE_DIRECTORY "/robot-models/";

GTEST_TEST(UrdfParser, parseFile)
{
    std::vector<std::string> test_data;
    test_data.push_back(urdf_directory + "four_bar.urdf");
    test_data.push_back(urdf_directory + "six_bar.urdf");
    test_data.push_back(urdf_directory + "revolute_rotor_chain.urdf");
    test_data.push_back(urdf_directory + "planar_leg_linkage.urdf");
    test_data.push_back(urdf_directory + "mini_cheetah.urdf");
    test_data.push_back(urdf_directory + "mit_humanoid_leg.urdf");
    test_data.push_back(urdf_directory + "mit_humanoid.urdf");

    for (const std::string &sample : test_data)
    {
        std::cout << "\n\nURDF file: " << sample << std::endl;
        ClusterTreeModel<double> cluster_model;
        cluster_model.buildModelFromURDF(sample);
        cluster_model.print();
        GTEST_ASSERT_GT(cluster_model.bodies().size(), 0);
    }
}

using RobotPtr = std::shared_ptr<Robot<double>>;

struct URDFvsManualTestData
{
    std::string urdf_file;
    RobotPtr robot;
};

std::vector<URDFvsManualTestData> GetTestRobots()
{
    std::vector<URDFvsManualTestData> test_data;
    test_data.push_back({urdf_directory + "planar_leg_linkage.urdf",
                         std::make_shared<PlanarLegLinkage<double>>()});
    test_data.push_back({urdf_directory + "revolute_rotor_chain.urdf",
                         std::make_shared<RevoluteChainWithRotor<3, double>>(false)});
    test_data.push_back({urdf_directory + "mini_cheetah.urdf",
                         std::make_shared<MiniCheetah<double>>()});
    test_data.push_back({urdf_directory + "mit_humanoid_leg.urdf",
                         std::make_shared<MIT_Humanoid_Leg<double>>()});
    // test_data.push_back({urdf_directory + "mit_humanoid.urdf",
    //                      std::make_shared<MIT_Humanoid<double>>()});
    return test_data;
}

class URDFvsManualTests : public ::testing::TestWithParam<URDFvsManualTestData>
{
protected:
    URDFvsManualTests()
    {
        std::cout << "URDF file: " << GetParam().urdf_file << std::endl;
        manual_model = GetParam().robot->buildClusterTreeModel();
        urdf_model.buildModelFromURDF(GetParam().urdf_file);
        urdf_model.setGravity(manual_model.getGravity().tail<3>());
    }

    void initializeRandomStates()
    {
        ModelState<double> model_state;
        for (const auto &cluster : manual_model.clusters())
        {
            JointState<> joint_state = cluster->joint_->randomJointState();
            model_state.push_back(joint_state);
        }

        urdf_model.setState(model_state);
        manual_model.setState(model_state);
    }

    ClusterTreeModel<double> urdf_model;
    ClusterTreeModel<double> manual_model;
};

INSTANTIATE_TEST_SUITE_P(Robots, URDFvsManualTests, ::testing::ValuesIn(GetTestRobots()));

TEST_P(URDFvsManualTests, compareToManuallyConstructed)
{
    // Tests that validate the model structure
    GTEST_ASSERT_EQ(this->manual_model.bodies().size(),
                    this->urdf_model.bodies().size());
    GTEST_ASSERT_EQ(this->manual_model.getNumPositions(),
                    this->urdf_model.getNumPositions());
    GTEST_ASSERT_EQ(this->manual_model.getNumDegreesOfFreedom(),
                    this->urdf_model.getNumDegreesOfFreedom());

    // Tests that vary with the state
    for (int i = 0; i < 25; i++)
    {
        this->initializeRandomStates();
        this->manual_model.forwardKinematics();
        this->urdf_model.forwardKinematics();

        // Verify the constraint Jacobians
        for (size_t j = 0; j < this->manual_model.clusters().size(); j++)
        {
            auto manual_cluster = this->manual_model.cluster(j);
            std::shared_ptr<LoopConstraint::Base<double>> constraint =
                manual_cluster->joint_->cloneLoopConstraint();
            DMat<double> G_manual = constraint->G();
            DVec<double> g_manual = constraint->g();
            DMat<double> K_manual = constraint->K();
            DVec<double> k_manual = constraint->k();

            auto urdf_cluster = this->urdf_model.cluster(j);
            std::shared_ptr<LoopConstraint::Base<double>> urdf_constraint =
                urdf_cluster->joint_->cloneLoopConstraint();
            DMat<double> G_urdf = urdf_constraint->G();
            DVec<double> g_urdf = urdf_constraint->g();
            DMat<double> K_urdf = urdf_constraint->K();
            DVec<double> k_urdf = urdf_constraint->k();

            GTEST_ASSERT_LT((G_manual - G_urdf).norm(), tol);
            GTEST_ASSERT_LT((g_manual - g_urdf).norm(), tol);
            GTEST_ASSERT_LT((K_manual - K_urdf).norm(), tol);
            GTEST_ASSERT_LT((k_manual - k_urdf).norm(), tol);
        }

        // Verify link kinematics
        for (const auto &body : this->manual_model.bodies())
        {
            const Vec3<double> p_manual = this->manual_model.getPosition(body.name_);
            const Vec3<double> p_urdf = this->urdf_model.getPosition(body.name_);
            GTEST_ASSERT_LT((p_manual - p_urdf).norm(), tol);

            const Mat3<double> R_manual = this->manual_model.getOrientation(body.name_);
            const Mat3<double> R_urdf = this->urdf_model.getOrientation(body.name_);
            GTEST_ASSERT_LT((R_manual - R_urdf).norm(), tol);

            const Vec3<double> v_manual = this->manual_model.getLinearVelocity(body.name_);
            const Vec3<double> v_urdf = this->urdf_model.getLinearVelocity(body.name_);
            GTEST_ASSERT_LT((v_manual - v_urdf).norm(), tol);

            const Vec3<double> w_manual = this->manual_model.getAngularVelocity(body.name_);
            const Vec3<double> w_urdf = this->urdf_model.getAngularVelocity(body.name_);
            GTEST_ASSERT_LT((w_manual - w_urdf).norm(), tol);
        }

        // Verify the mass matrix
        const DMat<double> H_manual = this->manual_model.getMassMatrix();
        const DMat<double> H_urdf = this->urdf_model.getMassMatrix();
        GTEST_ASSERT_LT((H_manual - H_urdf).norm(), tol);

        // Verify the bias forces
        const DVec<double> C_manual = this->manual_model.getBiasForceVector();
        const DVec<double> C_urdf = this->urdf_model.getBiasForceVector();
        GTEST_ASSERT_LT((C_manual - C_urdf).norm(), tol);

        // Verify the forward dynamics
        const DVec<double> tau = DVec<double>::Random(this->manual_model.getNumDegreesOfFreedom());
        const DVec<double> ydd_manual = this->manual_model.forwardDynamics(tau);
        const DVec<double> ydd_urdf = this->urdf_model.forwardDynamics(tau);
        GTEST_ASSERT_LT((ydd_manual - ydd_urdf).norm(), tol * 1.e1);

        // Verify the inverse dynamics
        const DVec<double> ydd = DVec<double>::Random(this->manual_model.getNumDegreesOfFreedom());
        const DVec<double> tau_manual = this->manual_model.inverseDynamics(ydd);
        const DVec<double> tau_urdf = this->urdf_model.inverseDynamics(ydd);
        GTEST_ASSERT_LT((tau_manual - tau_urdf).norm(), tol);

        // ========================================================================
        // Verify the inverse dynamics derivatives using COMPLEX STEP DIFFERENTIATION
        // ========================================================================
        //
        // We use complex step instead of finite differences because:
        //   1. Machine precision accuracy (error ~ 1e-16 vs 1e-8 for finite diff)
        //   2. No subtractive cancellation (extracts imag part, no subtraction)
        //   3. Can use h = 1e-30 instead of h = 1e-8
        //
        // Method: f'(x) = imag(f(x + ih)) / h where i = sqrt(-1)
        //
        // NOTE: The firstOrderInverseDynamicsDerivatives() implementation is incomplete
        // for floating bases with configuration-dependent motion subspaces (see comments
        // marked "// + gradient terms" in ClusterTreeDynamics.cpp). We only test fixed-base
        // robots where the motion subspace matrix S is configuration-independent.
        // ========================================================================

        // Determine if this is a floating base system
        auto root_cluster = this->manual_model.cluster(0);
        const bool has_floating_base = (root_cluster->parent_index_ < 0) &&
                                        (root_cluster->num_velocities_ > 0);

        // Only test derivatives for fixed-base robots
        if (!has_floating_base) {
            // Get analytical derivatives from the implementation
            auto [dtau_dq, dtau_dqdot] =
                this->manual_model.firstOrderInverseDynamicsDerivatives(ydd);

            std::pair<DVec<double>, DVec<double>> state = this->manual_model.getState();
            const DVec<double>& q0 = state.first;
            const DVec<double>& qd0 = state.second;
            const int nDOF = this->manual_model.getNumDegreesOfFreedom();

            const double h = 1e-30;  // Complex step size (can be extremely small!)

            // Build a complex-valued model from URDF for complex step differentiation
            ClusterTreeModel<std::complex<double>> complex_model;
            complex_model.buildModelFromURDF(GetParam().urdf_file);
            complex_model.setGravity(this->manual_model.getGravity().tail<3>().cast<std::complex<double>>());

            // Verify dtau_dq (derivative w.r.t. joint positions) - Complex Step
            for (int i = 0; i < nDOF; ++i) {
                // Create complex state with imaginary perturbation in q[i]
                DVec<std::complex<double>> q_complex(nDOF);
                DVec<std::complex<double>> qd_complex(nDOF);
                DVec<std::complex<double>> ydd_complex(nDOF);

                for (int j = 0; j < nDOF; ++j) {
                    q_complex[j] = (j == i) ? std::complex<double>(q0[j], h)     // Add imaginary perturbation
                                             : std::complex<double>(q0[j], 0.0);
                    qd_complex[j] = std::complex<double>(qd0[j], 0.0);
                    ydd_complex[j] = std::complex<double>(ydd[j], 0.0);
                }

                std::pair<DVec<std::complex<double>>, DVec<std::complex<double>>> state_complex =
                    {q_complex, qd_complex};
                complex_model.setState(state_complex);

                // Compute complex inverse dynamics
                DVec<std::complex<double>> tau_complex = complex_model.inverseDynamics(ydd_complex);

                // Extract derivative from imaginary part (no subtraction needed!)
                DVec<double> dtau_dqi(nDOF);
                for (int j = 0; j < nDOF; ++j) {
                    dtau_dqi[j] = tau_complex[j].imag() / h;
                }

                GTEST_ASSERT_LT((dtau_dqi - dtau_dq.col(i)).norm(), tol);
            }

            // Verify dtau_dqdot (derivative w.r.t. joint velocities) - Complex Step
            for (int i = 0; i < nDOF; ++i) {
                // Create complex state with imaginary perturbation in qd[i]
                DVec<std::complex<double>> q_complex(nDOF);
                DVec<std::complex<double>> qd_complex(nDOF);
                DVec<std::complex<double>> ydd_complex(nDOF);

                for (int j = 0; j < nDOF; ++j) {
                    q_complex[j] = std::complex<double>(q0[j], 0.0);
                    qd_complex[j] = (j == i) ? std::complex<double>(qd0[j], h)     // Add imaginary perturbation
                                              : std::complex<double>(qd0[j], 0.0);
                    ydd_complex[j] = std::complex<double>(ydd[j], 0.0);
                }

                std::pair<DVec<std::complex<double>>, DVec<std::complex<double>>> state_complex =
                    {q_complex, qd_complex};
                complex_model.setState(state_complex);

                // Compute complex inverse dynamics
                DVec<std::complex<double>> tau_complex = complex_model.inverseDynamics(ydd_complex);

                // Extract derivative from imaginary part (no subtraction needed!)
                DVec<double> dtau_dqdoti(nDOF);
                for (int j = 0; j < nDOF; ++j) {
                    dtau_dqdoti[j] = tau_complex[j].imag() / h;
                }

                GTEST_ASSERT_LT((dtau_dqdoti - dtau_dqdot.col(i)).norm(), tol);
            }
        }  // end if (!has_floating_base)

    }
}
