#include "gtest/gtest.h"

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
    Tello, TeleopArm,
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

struct URDFParserTestData
{
    std::string urdf_file;
    bool floating_base;  
};

GTEST_TEST(UrdfParser, parseFile)
{
    std::vector<URDFParserTestData> test_data;
    test_data.push_back({urdf_directory + "four_bar.urdf", false});
    test_data.push_back({urdf_directory + "six_bar.urdf", false});
    test_data.push_back({urdf_directory + "revolute_rotor_chain.urdf", false});
    test_data.push_back({urdf_directory + "planar_leg_linkage.urdf", false});
    test_data.push_back({urdf_directory + "mini_cheetah.urdf", true});
    test_data.push_back({urdf_directory + "mit_humanoid_leg.urdf", false});
    test_data.push_back({urdf_directory + "mit_humanoid.urdf", true});

    for (const URDFParserTestData &sample : test_data)
    {
        std::cout << "\n\nURDF file: " << sample.urdf_file << std::endl;
        ClusterTreeModel<double> cluster_model;
        cluster_model.buildModelFromURDF(sample.urdf_file, sample.floating_base);
        cluster_model.print();
        GTEST_ASSERT_GT(cluster_model.bodies().size(), 0);
    }
}

using RobotPtr = std::shared_ptr<Robot<double>>;

struct URDFvsManualTestData
{
    std::string urdf_file;
    RobotPtr robot;
    bool floating_base;  
};

std::vector<URDFvsManualTestData> GetTestRobots()
{
    std::vector<URDFvsManualTestData> test_data;
    test_data.push_back({urdf_directory + "planar_leg_linkage.urdf",
                         std::make_shared<PlanarLegLinkage<double>>(),
                         false});
    test_data.push_back({urdf_directory + "revolute_rotor_chain.urdf",
                         std::make_shared<RevoluteChainWithRotor<3, double>>(false),
                         false});
    test_data.push_back({urdf_directory + "mini_cheetah.urdf",
                         std::make_shared<MiniCheetah<double>>(),
                         true});
    test_data.push_back({urdf_directory + "mit_humanoid_leg.urdf",
                         std::make_shared<MIT_Humanoid_Leg<double>>(),
                         false});
    test_data.push_back({urdf_directory + "mit_humanoid.urdf",
                         std::make_shared<MIT_Humanoid<double>>(),
                         true});
    return test_data;
}

class URDFvsManualTests : public ::testing::TestWithParam<URDFvsManualTestData>
{
protected:
    URDFvsManualTests()
    {
        std::cout << "URDF file: " << GetParam().urdf_file << std::endl;
        manual_model = GetParam().robot->buildClusterTreeModel();
        urdf_model.buildModelFromURDF(GetParam().urdf_file, GetParam().floating_base);
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
    }
}
