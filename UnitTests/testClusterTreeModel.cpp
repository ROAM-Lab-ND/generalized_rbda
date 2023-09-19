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
