#include "grbda/Dynamics/ClusterTreeModel.h"

#include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

int main()
{
    const std::string urdf_filename = "../robot-models/four_bar.urdf";

    // Pinocchio model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // GRBDA model
    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);
    const int nq = cluster_tree.getNumPositions();
    const int nv = cluster_tree.getNumDegreesOfFreedom();
    
    // Loop constraint via GRBDA model
    const auto cluster = cluster_tree.clusters()[0];
    std::shared_ptr<grbda::LoopConstraint::Base<double>> constraint = cluster->joint_->cloneLoopConstraint();

    // Loop constraint via manual implementation
    std::vector<double> path1_lengths, path2_lengths;
    path1_lengths.push_back(0.5);
    path1_lengths.push_back(1.0);
    path2_lengths.push_back(0.5);
    Eigen::Vector2d offset = Eigen::Vector2d(1.0, 0.0);
    int independent_coordinate = 0;
    std::shared_ptr<grbda::LoopConstraint::FourBar<double>> loop_constraint =
        std::make_shared<grbda::LoopConstraint::FourBar<double>>(path1_lengths, path2_lengths, offset, independent_coordinate);

    // Sample joint positions
    std::vector<Eigen::Vector3d> q_samples;
    q_samples.push_back(Eigen::Vector3d(0., 0., 0.));
    q_samples.push_back(Eigen::Vector3d(1.57, 0., 1.57));
    q_samples.push_back(Eigen::Vector3d(1.57, -1.57, 1.57));
    q_samples.push_back(Eigen::Vector3d(-1.57, 1.57, -1.57));

    // Test the implicit loop constraint
    for (const auto &q : q_samples)
    {
        grbda::JointCoordinate<double> joint_pos(q, true);
        std::cout << "urdf phi(" << q.transpose() << "): "
                  << constraint->phi(joint_pos).transpose() << std::endl;

        std::cout << "manual phi(" << q.transpose() << "): "
                  << loop_constraint->phi(joint_pos).transpose() << std::endl;
    }

    return 0;
}
