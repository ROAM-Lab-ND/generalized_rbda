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
    grbda::ModelState<double> spanning_model_state;

    Eigen::MatrixXd K_cluster = Eigen::MatrixXd::Zero(0,0);
    Eigen::VectorXd k_cluster = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd G_transpose_cluster = Eigen::MatrixXd::Zero(0,0);

    for (const auto &cluster : cluster_tree.clusters())
    {
        grbda::JointState<double> joint_state = cluster->joint_->randomJointState();
        grbda::JointState<double> spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

        model_state.push_back(joint_state);
        spanning_model_state.push_back(spanning_joint_state);

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

    //std::cout << "cluster tau verification: " << tau_2 << std::endl;

    std::cout << "K cluster: \n" << K_cluster << std::endl;
    std::cout << "k_cluster: \n" << k_cluster << std::endl;
    std::cout << "G_transpose_cluster: \n" << G_transpose_cluster << std::endl;
    std::cout << "G_transpose_pinv_cluster: \n" << G_transpose_pinv << std::endl;

    cluster_tree.setState(model_state);
    auto spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
    Eigen::VectorXd spanning_joint_pos = spanning_q_and_v.first;
    Eigen::VectorXd spanning_joint_vel = spanning_q_and_v.second;

    // Start with the forward kinematics
    grbda::DMat<double> joint_map = cluster_tree.getPinocchioJointMap();
    Eigen::VectorXd pin_q(6), pin_v(6), pin_tau(6);
    pin_q = joint_map * spanning_joint_pos;
    pin_v = joint_map * spanning_joint_vel;
    pin_tau = joint_map * spanning_joint_tau;

    pinocchio::forwardKinematics(model, data, pin_q, pin_v);
    cluster_tree.forwardKinematics();

    std::cout << "Pinocchio Model Joints Names" << std::endl;
    for (int i = 0; i < model.njoints; i++)
    {
        std::cout << model.names[i] << std::endl;
    }

    // Joint 1 Velocity
    std::cout << "Joint 1 Velocity (model)       : " << data.v[1].angular().transpose() << " " << data.v[1].linear().transpose() << std::endl;
    std::cout << "Joint 1 Velocity (cluster_tree): " << cluster_tree.cluster(0)->v_.head<6>().transpose() << std::endl;

    // Joint 2 Velocity
    std::cout << "Joint 2 Velocity (model)       : " << data.v[2].angular().transpose() << " " << data.v[2].linear().transpose() << std::endl;
    std::cout << "Joint 2 Velocity (cluster_tree): " << cluster_tree.cluster(1)->v_.head<6>().transpose() << std::endl;

    // Joint 3 Velocity
    std::cout << "Joint 3 Velocity (model)       : " << data.v[3].angular().transpose() << " " << data.v[3].linear().transpose() << std::endl;
    std::cout << "Joint 3 Velocity (cluster_tree): " << cluster_tree.cluster(2)->v_.head<6>().transpose() << std::endl;

    const Eigen::VectorXd qdd_cluster = cluster_tree.forwardDynamics(tau);
    std::cout << "qdd_cluster: \n" << qdd_cluster << std::endl;

    const double mu0 = 1e-12;
    pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau, K_cluster, k_cluster, mu0);
    std::cout << "qdd_pinocchio: \n" << data.ddq << std::endl;
    
    lg_mult_eigen_model.setState(spanning_joint_pos, spanning_joint_vel);
    const Eigen::VectorXd qdd_lg_eigen_full = lg_mult_eigen_model.forwardDynamics(tau);
    std::cout << "qdd_lg_eigen_full: \n" << qdd_lg_eigen_full << std::endl;

    return 0;
}
