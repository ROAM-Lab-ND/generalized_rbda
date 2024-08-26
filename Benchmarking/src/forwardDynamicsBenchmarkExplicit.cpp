#include "gtest/gtest.h"

#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

GTEST_TEST(pinocchio, explicit_constraints)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;

    // Build models
    const std::string urdf_filename = "../robot-models/revolute_rotor_chain.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);
    Eigen::MatrixXd joint_map = cluster_tree.getPinocchioJointMap();

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);

    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    // Create a random state
    ModelState model_state;
    ModelState spanning_model_state;
    for (const auto &cluster : cluster_tree.clusters())
    {
        JointState joint_state = cluster->joint_->randomJointState();
        JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);
        model_state.push_back(joint_state);
        spanning_model_state.push_back(spanning_joint_state);
    }

    auto spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
    Eigen::VectorXd spanning_joint_pos = spanning_q_and_v.first;
    Eigen::VectorXd spanning_joint_vel = spanning_q_and_v.second;
    cluster_tree.setState(model_state);
    lgm_model.setState(spanning_joint_pos, spanning_joint_vel);

    Eigen::VectorXd pin_q(nv_span), pin_v(nv_span);
    pin_q = joint_map * spanning_joint_pos;
    pin_v = joint_map * spanning_joint_vel;

    // Extract loop constraints from the cluster tree
    // TODO(@MatthewChignoli): Sometimes we need to update kinematics before extracting G and K
    Eigen::MatrixXd K_cluster = Eigen::MatrixXd::Zero(0, 0);
    Eigen::VectorXd k_cluster = Eigen::VectorXd::Zero(0);
    Eigen::MatrixXd G_cluster = Eigen::MatrixXd::Zero(0, 0);
    Eigen::VectorXd g_cluster = Eigen::VectorXd::Zero(0);
    for (const auto &cluster : cluster_tree.clusters())
    {
        K_cluster = grbda::appendEigenMatrix(K_cluster, cluster->joint_->K());
        k_cluster = grbda::appendEigenVector(k_cluster, cluster->joint_->k());
        G_cluster = grbda::appendEigenMatrix(G_cluster, cluster->joint_->G());
        g_cluster = grbda::appendEigenVector(g_cluster, cluster->joint_->g());
    }

    // Convert implicit loop constraint to Pinocchio joint order
    const Eigen::MatrixXd K_pinocchio = K_cluster * joint_map.transpose();
    const Eigen::VectorXd k_pinocchio = k_cluster;

    // Forward kinematics
    pinocchio::forwardKinematics(model, data, pin_q, pin_v);
    cluster_tree.forwardKinematics();

    // TODO(@MatthewChignoli): This is the only part that is not automated
    for (int i : {1, 2, 3})
    {
        Eigen::Vector<double, 6> pinocchio_link_velocity;
        pinocchio_link_velocity << data.v[i].angular(), data.v[i].linear();

        Eigen::Vector<double, 6> cluster_link_velocity = cluster_tree.cluster(i - 1)->v_.head<6>();

        GTEST_ASSERT_LT((pinocchio_link_velocity - cluster_link_velocity).norm(), 1e-12)
            << "Link " << i - 1 << " velocities do not match";
    }

    // Compute the forward dynamics
    Eigen::VectorXd spanning_joint_tau = Eigen::VectorXd::Random(nv_span);
    Eigen::VectorXd tau = G_cluster.transpose() * spanning_joint_tau;

    const double mu0 = 1e-12;
    Eigen::VectorXd pin_tau(nv_span);
    pin_tau = joint_map * spanning_joint_tau;

    const Eigen::VectorXd ydd_cluster = cluster_tree.forwardDynamics(tau);
    pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau, K_pinocchio, k_pinocchio, mu0);
    const Eigen::VectorXd qdd_grbda = lgm_model.forwardDynamics(tau);

    // Check grbda cluster tree solution against lagrange multiplier solution
    const Eigen::VectorXd ydd_error = qdd_grbda - (G_cluster * ydd_cluster + g_cluster);
    GTEST_ASSERT_LT(ydd_error.norm(), 1e-10)
        << "ydd_cluster: " << ydd_cluster.transpose() << "\n"
        << "G*qdd_grbda + g: " << (G_cluster * qdd_grbda + g_cluster).transpose();

    // Check grbda lagrange multiplier solution against pinocchio
    const Eigen::VectorXd qdd_error = data.ddq - joint_map * qdd_grbda;
    GTEST_ASSERT_LT(qdd_error.norm(), 1e-10)
        << "qdd_pinocchio   : " << data.ddq.transpose() << "\n"
        << "jmap * qdd_grbda: " << (joint_map * qdd_grbda).transpose();

    // Check that solutions satisfy the loop constraints
    Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data.ddq + k_pinocchio;
    Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_grbda + k_cluster;

    GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), 1e-10)
        << "K*qdd_grbda + k    : " << cnstr_violation_grbda.transpose();

    GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), 1e-10)
        << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
}
