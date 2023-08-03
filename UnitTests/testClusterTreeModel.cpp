#include "gtest/gtest.h"

#include "Dynamics/ClusterTreeModel.h"
#include "Robots/RobotTypes.h"

using namespace grbda;

static const double tol = 1e-10;

template <class T>
class ClusterTreeModelTest : public testing::Test
{
protected:
    ClusterTreeModelTest() : cluster_model(robot.buildClusterTreeModel()) {}

    T robot;
    ClusterTreeModel cluster_model;
};

using testing::Types;

typedef Types<
    RevoluteChainWithRotor<4>,
    RevolutePairChainWithRotor<4>,
    RevoluteChainMultipleRotorsPerLink<4, 3>,
    RevoluteChainWithAndWithoutRotor<4ul, 4ul>,
    Tello, TeleopArm, MIT_Humanoid, MiniCheetah>
    Robots;

TYPED_TEST_SUITE(ClusterTreeModelTest, Robots);

TYPED_TEST(ClusterTreeModelTest, EndEffectors)
{
    const int nv = this->cluster_model.getNumDegreesOfFreedom();
    const int num_clusters = this->cluster_model.clusters().size();
    const int num_ee = this->cluster_model.contactPoints().size();

    for (const auto &ee : this->cluster_model.endEffectors())
    {
        ASSERT_EQ(ee.ChiUp_.size(), num_clusters);
    }

    std::vector<int> supported_end_effectors;
    for (const auto &cluster : this->cluster_model.clusters())
    {
        for (const auto &ee_index : cluster->supported_end_effectors_)
        {
            // Check if contact point is already accounted for
            auto it = std::find(supported_end_effectors.begin(),
                                supported_end_effectors.end(), ee_index);
            // If not, push it back
            if (it == supported_end_effectors.end())
                supported_end_effectors.push_back(ee_index);
        }
    }
    ASSERT_EQ(supported_end_effectors.size(), this->cluster_model.endEffectors().size());
}

TYPED_TEST(ClusterTreeModelTest, SetState)
{
    // This test ensures that the ClusterTreeModel::setState method and the 
    // ClusterTreeModel::initializeState method produce the same results

    // Create a random model state
    ModelState model_state;
    DVec<double> q(0);
    DVec<double> qd(0);
    for (const auto &cluster : this->cluster_model.clusters())
    {
        JointState joint_state;
        bool nan_in_joint_state = true;
        while (nan_in_joint_state)
        {
            joint_state = cluster->joint_->randomJointState();
            if (!joint_state.position.hasNaN())
            {
                nan_in_joint_state = false;
            }
        }

        model_state.push_back(cluster->joint_->randomJointState());
        q = appendEigenVector(q, model_state.back().position);
        qd = appendEigenVector(qd, model_state.back().velocity);
    }
    const int &nq = this->cluster_model.getNumPositions();
    const int &nv = this->cluster_model.getNumDegreesOfFreedom();
    DVec<double> state(nq + nv);
    state << q, qd;

    const int num_samples = 100;
    for (int j(0); j < num_samples; j++)
    {
        // Compute kinematic quantities from setState
        this->cluster_model.setState(state);
        this->cluster_model.forwardKinematics();

        std::vector<DVec<double>> link_velocities_set_state;
        for (const auto &cluster : this->cluster_model.clusters())
        {
            link_velocities_set_state.push_back(cluster->v_);
        }

        std::vector<DVec<double>> cp_positions_set_state;
        std::vector<DVec<double>> cp_velocities_set_state;
        for (const auto &cp : this->cluster_model.contactPoints())
        {
            cp_positions_set_state.push_back(cp.position_);
            cp_velocities_set_state.push_back(cp.velocity_);
        }

        // Compute kinematic quantities from initializeState
        this->cluster_model.initializeState(model_state);
        this->cluster_model.forwardKinematics();

        std::vector<DVec<double>> link_velocities_initialize_state;
        for (const auto &cluster : this->cluster_model.clusters())
        {
            link_velocities_initialize_state.push_back(cluster->v_);
        }

        std::vector<DVec<double>> cp_positions_initialize_state;
        std::vector<DVec<double>> cp_velocities_initialize_state;
        for (const auto &cp : this->cluster_model.contactPoints())
        {
            cp_positions_initialize_state.push_back(cp.position_);
            cp_velocities_initialize_state.push_back(cp.velocity_);
        }

        // Enforce agreement
        for (size_t i(0); i < link_velocities_set_state.size(); i++)
        {
            DVec<double> error = link_velocities_set_state[i] - link_velocities_initialize_state[i];
            GTEST_ASSERT_LT(error.norm(), tol);
        }

        for (size_t i(0); i < cp_positions_set_state.size(); i++)
        {
            DVec<double> pos_error = cp_positions_set_state[i] - cp_positions_initialize_state[i];
            DVec<double> vel_error = cp_velocities_set_state[i] - cp_velocities_initialize_state[i];

            GTEST_ASSERT_LT(pos_error.norm(), tol);
            GTEST_ASSERT_LT(vel_error.norm(), tol);
        }
    }
}
