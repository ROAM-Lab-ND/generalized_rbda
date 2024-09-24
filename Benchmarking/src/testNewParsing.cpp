#include "pinocchioHelpers.hpp"

GTEST_TEST(urdf_parsing_tests, pinocchio)
{
    // TODO(@MatthewChignoli): Now validate the dynamics?

    {
        std::string urdf_filename = main_urdf_directory + "simple_humanoid.urdf";
        std::cout << "Urdf: " << urdf_filename << std::endl;

        // Pinocchio model
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
        pinocchio::Data data(model);

        // Cluster tree model
        grbda::ClusterTreeModel<double> cluster_tree;
        cluster_tree.buildModelFromURDF(urdf_filename, false);

        GTEST_ASSERT_EQ(model.nq, cluster_tree.getNumPositions());
        GTEST_ASSERT_EQ(model.nv, cluster_tree.getNumDegreesOfFreedom());
    }

    {
        std::string urdf_filename = main_urdf_directory + "double_pendulum.urdf";
        std::cout << "Urdf: " << urdf_filename << std::endl;

        // Pinocchio model
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
        pinocchio::Data data(model);

        // Cluster tree model
        grbda::ClusterTreeModel<double> cluster_tree;
        cluster_tree.buildModelFromURDF(urdf_filename, false);

        GTEST_ASSERT_EQ(model.nq, cluster_tree.getNumPositions());
        GTEST_ASSERT_EQ(model.nv, cluster_tree.getNumDegreesOfFreedom());

        GTEST_ASSERT_EQ(model.nq, cluster_tree.getNumPositions());
        GTEST_ASSERT_EQ(model.nv, cluster_tree.getNumDegreesOfFreedom());
    }
}
