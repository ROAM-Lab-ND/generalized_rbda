#include "gtest/gtest.h"

#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

GTEST_TEST(UrdfParser, parseFile)
{
    // TODO(@MatthewChignoli): Make sure test works for all of these files
    std::vector<std::string> urdf_files;
    // urdf_files.push_back("/home/matt/repos/URDF-Parser/four_bar.urdf");
    // urdf_files.push_back("/home/matt/repos/URDF-Parser/six_bar.urdf");
    urdf_files.push_back("/home/matt/repos/URDF-Parser/planar_leg_linkage.urdf");
    // urdf_files.push_back("/home/matt/repos/URDF-Parser/mini_cheetah.urdf");

    for (const std::string &urdf_file : urdf_files)
    {
        ClusterTreeModel<double> cluster_model;
        cluster_model.buildModelFromURDF(urdf_file);
        std::cout << "URDF file: " << urdf_file << std::endl;
        cluster_model.print();
        GTEST_ASSERT_GT(cluster_model.bodies().size(), 0);
    }
}

class URDFvsManualTests : public ::testing::Test
{
protected:
    URDFvsManualTests() : manual_model(robot.buildClusterTreeModel())
    {
        urdf_model.buildModelFromURDF("/home/matt/repos/URDF-Parser/planar_leg_linkage.urdf");
    }

    void initializeRandomStates()
    {
        model_state.clear();
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
        for (const auto &cluster : manual_model.clusters())
        {
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

    PlanarLegLinkage<double> robot;
    ClusterTreeModel<double> urdf_model;
    ClusterTreeModel<double> manual_model;

    ModelState<double> model_state;
};

static const double tol = 1e-10;

// TODO(@MatthewChignoli): Add the rolling without slipping constraint to the URDF model

// TODO(@MatthewChignoli): This is basicaly a direct copt of the testForwardKinematics unit test. Any way to reduce code duplication?
TEST_F(URDFvsManualTests, compareToManuallyConstructed)
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
