#include "grbda/Dynamics/ClusterTreeModel.h"

#include "pinocchio/parsers/urdf.hpp"
//#include "pinocchio/algorithm/joint-configuration.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"
//#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"


int main()
{
    const std::string urdf_filename = "../robot-models/four_bar.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);

    const int nq = cluster_tree.getNumPositions();
    const int nv = cluster_tree.getNumDegreesOfFreedom();

    grbda::ModelState<double> model_state;
    Eigen::VectorXd spanning_joint_pos = Eigen::VectorXd::Zero(0);
    Eigen::VectorXd spanning_joint_vel = Eigen::VectorXd::Zero(0);

    Eigen::Vector3d q;
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    const auto cluster = cluster_tree.clusters()[0];
    std::shared_ptr<grbda::LoopConstraint::Base<double>> constraint = cluster->joint_->cloneLoopConstraint();

    q << 0., 0., 0.;
    grbda::JointCoordinate<double> joint_pos(q, true);
    std::cout << "phi([0, 0, 0]): " << constraint->phi(joint_pos) << std::endl;

    q << 1.57, 0., 0.;
    grbda::JointCoordinate<double> joint_pos_2(q, true);
    std::cout << "phi([1.57, 0, 0]): " << constraint->phi(joint_pos_2) << std::endl;

    q << -1.57, 0., 0.;
    grbda::JointCoordinate<double> joint_pos_3(q, true);
    std::cout << "phi([-1.57, 0, 0]): " << constraint->phi(joint_pos_3) << std::endl;

    q << 0., 1.57, 0.;
    grbda::JointCoordinate<double> joint_pos_4(q, true);
    std::cout << "phi([0, 1.57, 0]): " << constraint->phi(joint_pos_4) << std::endl;

    q << 0., -1.57, 0.;
    grbda::JointCoordinate<double> joint_pos_5(q, true);
    std::cout << "phi([0, -1.57, 0]): " << constraint->phi(joint_pos_5) << std::endl;

    return 0;
}
