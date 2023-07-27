#include "gtest/gtest.h"

#include "Dynamics/ClusterTreeModel.h"
#include "Robots/RobotTypes.h"

using namespace grbda;
using namespace grbda::ClusterNodeVisitors;

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
    Tello, TeleopArm>
    Robots;

TYPED_TEST_SUITE(ClusterTreeModelTest, Robots);

TYPED_TEST(ClusterTreeModelTest, SetState)
{
    // This test ensures that the ClusterTreeModel::setState method and the 
    // ClusterTreeModel::initializeState method produce the same results

    // Create a random model state
    ModelState model_state;
    DVec<double> q(0);
    DVec<double> qd(0);
    for (const ClusterTreeModel::NodeTypeVariants &cluster : this->cluster_model.clusterVariants())
    {
        model_state.push_back(getJoint(cluster)->randomJointState());
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
        for (const ClusterTreeModel::NodeTypeVariants &cluster : this->cluster_model.clusterVariants())
        {
            link_velocities_set_state.push_back(velocity(cluster));
        }

        std::vector<DVec<double>> cp_positions_set_state;
        std::vector<DVec<double>> cp_velocities_set_state;
        for (const ContactPoint &cp : this->cluster_model.contactPoints())
        {
            cp_positions_set_state.push_back(cp.position_);
            cp_velocities_set_state.push_back(cp.velocity_);
        }

        // Compute kinematic quantities from initializeState
        this->cluster_model.initializeState(model_state);
        this->cluster_model.forwardKinematics();

        std::vector<DVec<double>> link_velocities_initialize_state;
        for (const ClusterTreeModel::NodeTypeVariants &cluster : this->cluster_model.clusterVariants())
        {
            link_velocities_initialize_state.push_back(velocity(cluster));
        }

        std::vector<DVec<double>> cp_positions_initialize_state;
        std::vector<DVec<double>> cp_velocities_initialize_state;
        for (const ContactPoint &cp : this->cluster_model.contactPoints())
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
