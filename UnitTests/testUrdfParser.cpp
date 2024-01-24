#include "gtest/gtest.h"

#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

GTEST_TEST(UrdfParser, parseFile)
{
    std::vector<std::string> urdf_files;
    urdf_files.push_back("/home/matt/repos/URDF-Parser/four_bar.urdf");
    urdf_files.push_back("/home/matt/repos/URDF-Parser/six_bar.urdf");
    urdf_files.push_back("/home/matt/repos/URDF-Parser/planar_leg_linkage.urdf");
    urdf_files.push_back("/home/matt/repos/URDF-Parser/revolute_rotor_chain.urdf");

    // TODO(@MatthewChignoli): Add these later
    // urdf_files.push_back("/home/matt/repos/URDF-Parser/mini_cheetah_leg.urdf");
    // urdf_files.push_back("/home/matt/repos/URDF-Parser/mini_cheetah.urdf");

    for (const std::string &urdf_file : urdf_files)
    {
        ClusterTreeModel<double> cluster_model;
        cluster_model.buildModelFromURDF(urdf_file, false);
        std::cout << "\n\nURDF file: " << urdf_file << std::endl;
        cluster_model.print();
        GTEST_ASSERT_GT(cluster_model.bodies().size(), 0);
    }
}

using RobotPtr = std::shared_ptr<Robot<double>>;

std::vector<std::pair<std::string, RobotPtr>> GetTestRobots()
{
    std::vector<std::pair<std::string, RobotPtr>> robots;
    robots.push_back(std::make_pair("/home/matt/repos/URDF-Parser/planar_leg_linkage.urdf",
                                    std::make_shared<PlanarLegLinkage<double>>()));
    robots.push_back(std::make_pair("/home/matt/repos/URDF-Parser/revolute_rotor_chain.urdf",
                                    std::make_shared<RevoluteChainWithRotor<3, double>>()));
    return robots;
}

class URDFvsManualTests : public ::testing::TestWithParam<std::pair<std::string, RobotPtr>>
{
protected:
    URDFvsManualTests()
    {
        urdf_model.buildModelFromURDF(GetParam().first, false);
        manual_model = GetParam().second->buildClusterTreeModel();
    }

    void initializeRandomStates()
    {
        model_state.clear();
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
        for (const auto &cluster : manual_model.clusters())
        {
            // TODO(@MatthewChignoli): The gneric cluster should override the randomJointState function. Maybe that means we need different generic clusters for implicit vs. explicit?
            JointState<> joint_state = cluster->joint_->randomJointState();
            JointState<> spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                   spanning_joint_state.position);
            spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                   spanning_joint_state.velocity);
            model_state.push_back(joint_state);
        }

        urdf_model.setState(model_state);
        manual_model.setState(model_state);
    }

    ClusterTreeModel<double> urdf_model;
    ClusterTreeModel<double> manual_model;

    ModelState<double> model_state;
};

INSTANTIATE_TEST_SUITE_P(Robots, URDFvsManualTests, ::testing::ValuesIn(GetTestRobots()));

static const double tol = 1e-10;

TEST_P(URDFvsManualTests, compareToManuallyConstructed)
{
    // Make sure the robots have the same number of bodies
    GTEST_ASSERT_EQ(this->manual_model.bodies().size(), this->urdf_model.bodies().size());

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

        // TODO(@MatthewChignoli): Still need to compare the dynamics
    }
}
