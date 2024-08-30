#include "gtest/gtest.h"

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"

#include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> jointMap(
    const grbda::RigidBodyTreeModel<Scalar> &grbda_model,
    const pinocchio::Model &pin_model)
{
    // TODO(@MatthewChignoli): Assumes nq = nv, and nv_i = 1 for all joints
    using EigMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    EigMat joint_map = EigMat::Zero(grbda_model.getNumDegreesOfFreedom(),
                                    grbda_model.getNumDegreesOfFreedom());
    int i = 0;
    for (const auto &name : pin_model.names)
    {
        for (const auto &node : grbda_model.rigidBodyNodes())
        {
            if (node->joint_->name_ == name)
            {
                joint_map(i, node->velocity_index_) = 1;
                i++;
            }
        }
    }
    return joint_map;
}

const std::string urdf_directory = SOURCE_DIRECTORY "/robot-models/";

struct RobotSpecification
{
    std::string urdf_filename;
    bool floating_base;
};

std::vector<RobotSpecification> GetBenchmarkUrdfFiles()
{
    std::vector<RobotSpecification> test_urdf_files;
    test_urdf_files.push_back({urdf_directory + "four_bar.urdf", false});
    //test_urdf_files.push_back({urdf_direectory + "six_bar.urdf", false});
    return test_urdf_files;
}

class PinocchioImplicitNumericalValidation : public ::testing::TestWithParam<RobotSpecification>
{
};

INSTANTIATE_TEST_SUITE_P(PinocchioImplicitNumericalValidation, PinocchioImplicitNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioImplicitNumericalValidation, implicit_constraints)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;

    // Pinocchio model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(GetParam().urdf_filename, model);
    pinocchio::Data data(model);

    // GRBDA model
    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam().urdf_filename, GetParam().floating_base);
        
    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
    Eigen::MatrixXd joint_map = jointMap(lgm_model, model);

    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    // Sanity check with a valid spanning joint state
    ModelState spanning_model_state;
    JointState spanning_joint_state(true,true);
    Eigen::VectorXd spanning_joint_pos(3);
    spanning_joint_pos << -1.57, 1.57, -1.57;
    Eigen::VectorXd spanning_joint_vel = Eigen::VectorXd::Zero(3);
    spanning_joint_state.position = spanning_joint_pos;
    spanning_joint_state.velocity = spanning_joint_vel;
    spanning_model_state.push_back(spanning_joint_state);

    // Set state
    cluster_tree.setState(spanning_model_state);
    lgm_model.setState(spanning_joint_pos, spanning_joint_vel);

    // Get pinocchio state
    Eigen::VectorXd pin_q(nv_span), pin_v(nv_span);
    pin_q = joint_map * spanning_joint_pos;
    pin_v = joint_map * spanning_joint_vel;

    // Loop constraint via GRBDA model
    const auto cluster = cluster_tree.clusters()[0];
    std::shared_ptr<grbda::LoopConstraint::Base<double>> constraint = cluster->joint_->cloneLoopConstraint();

    // Update Jacobians and biases
    constraint->updateJacobians(spanning_joint_state.position);
    constraint->updateBiases(spanning_joint_state);

    /*// Loop constraint via manual implementation
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
    }*/

    // Extract loop constraints from the cluster tree
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

    std::cout << "K_: \n" << K_cluster << std::endl;
    std::cout << "G_: \n" << G_cluster << std::endl;
    std::cout << "k_: \n" << k_cluster << std::endl;
    std::cout << "g_: \n" << g_cluster << std::endl;

    // Convert implicit loop constraint to Pinocchio joint order
    const Eigen::MatrixXd K_pinocchio = K_cluster * joint_map.transpose();
    const Eigen::VectorXd k_pinocchio = k_cluster;

    // Forward kinematics
    // TODO(@MatthewChignoli): Add this test back in?
    // pinocchio::forwardKinematics(model, data, pin_q, pin_v);
    // cluster_tree.forwardKinematics();

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
