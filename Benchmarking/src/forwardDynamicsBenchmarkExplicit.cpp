#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"


int main()
{
    const std::string urdf_filename = "../robot-models/revolute_rotor_chain.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);

    grbda::RigidBodyTreeModel<double> lg_mult_eigen_model(
        cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
    
    const int nq = cluster_tree.getNumPositions();
    const int nv = cluster_tree.getNumDegreesOfFreedom();

    grbda::ModelState<double> model_state;
    Eigen::VectorXd spanning_joint_pos = Eigen::VectorXd::Zero(0);
    Eigen::VectorXd spanning_joint_vel = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd K_cluster = Eigen::MatrixXd::Zero(0,0);
    Eigen::VectorXd k_cluster = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd G_transpose_cluster = Eigen::MatrixXd::Zero(0,0);

    for (const auto &cluster : cluster_tree.clusters())
    {
        grbda::JointState<double> joint_state = cluster->joint_->randomJointState();
        grbda::JointState<double> spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

        spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                               spanning_joint_state.position);
        spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                               spanning_joint_state.velocity);

        model_state.push_back(joint_state);
        std::cout << "cluster_pos: " << joint_state.position << std::endl;
        std::cout << "cluster_vel: " << joint_state.velocity << std::endl;

        K_cluster = grbda::appendEigenMatrix(K_cluster, cluster->joint_->K());
        k_cluster = grbda::appendEigenVector(k_cluster, cluster->joint_->k());

        Eigen::MatrixXd G_transpose = cluster->joint_->G().transpose();
        G_transpose_cluster = grbda::appendEigenMatrix(G_transpose_cluster, G_transpose);
    }

    Eigen::MatrixXd G_transpose_pinv = grbda::matrixRightPseudoInverse(G_transpose_cluster);

    Eigen::VectorXd tau = Eigen::VectorXd::Random(nv);
    Eigen::VectorXd spanning_joint_tau = G_transpose_pinv * tau;
    //Eigen::VectorXd tau_2 = G_transpose_cluster * spanning_joint_tau;

    std::cout << "cluster_tau: \n" << tau << std::endl;

    std::cout << "spanning_joint_pos: \n" << spanning_joint_pos << std::endl;
    std::cout << "spanning_joint_vel: \n" << spanning_joint_vel << std::endl;
    std::cout << "spanning_joint_tau: \n" << spanning_joint_tau << std::endl;

    //std::cout << "cluster tau verification: " << tau_2 << std::endl;

    std::cout << "K cluster: \n" << K_cluster << std::endl;
    std::cout << "k_cluster: \n" << k_cluster << std::endl;
    std::cout << "G_transpose_cluster: \n" << G_transpose_cluster << std::endl;
    std::cout << "G_transpose_pinv_cluster: \n" << G_transpose_pinv << std::endl;

    cluster_tree.setState(model_state);

    const Eigen::VectorXd qdd_cluster = cluster_tree.forwardDynamics(tau);
    std::cout << "qdd_cluster: \n" << qdd_cluster << std::endl;

    const double mu0 = 1e-12;
    pinocchio::forwardDynamics(model, data, spanning_joint_pos, spanning_joint_vel,
        spanning_joint_tau, K_cluster, k_cluster, mu0);
    std::cout << "qdd_pinocchio: \n" << data.ddq << std::endl;
    
    lg_mult_eigen_model.setState(spanning_joint_pos, spanning_joint_vel);
    const Eigen::VectorXd qdd_lg_eigen_full = lg_mult_eigen_model.forwardDynamics(tau);
    std::cout << "qdd_lg_eigen_full: \n" << qdd_lg_eigen_full << std::endl;

    return 0;
}
