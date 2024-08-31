#include "gtest/gtest.h"

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"
#include "grbda/Utils/Utilities.h"

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"
#include "pinocchio/parsers/urdf.hpp"
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
            if (node->joint_->name() == name)
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
    test_urdf_files.push_back({urdf_directory + "revolute_rotor_chain.urdf", false});
    test_urdf_files.push_back({urdf_directory + "mit_humanoid.urdf", false});
    test_urdf_files.push_back({urdf_directory + "mini_cheetah.urdf", false});
    return test_urdf_files;
}

class PinocchioNumericalValidation : public ::testing::TestWithParam<RobotSpecification>
{
};

INSTANTIATE_TEST_SUITE_P(PinocchioNumericalValidation, PinocchioNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioNumericalValidation, forward_dynamics)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    // Build models
    pinocchio::Model model;
    pinocchio::urdf::buildModel(GetParam().urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam().urdf_filename, GetParam().floating_base);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::Projection);
    Eigen::MatrixXd joint_map = jointMap(lgm_model, model);

    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    for (int i = 0; i < 25; i++)
    {
        // Create a random state
        ModelState model_state;
        ModelState spanning_model_state;
        using LoopConstraint = grbda::LoopConstraint::Base<double>;
        for (const auto &cluster : cluster_tree.clusters())
        {
            JointState joint_state = cluster->joint_->randomJointState();
            JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            std::shared_ptr<LoopConstraint> constraint = cluster->joint_->cloneLoopConstraint();
            if (!constraint->isExplicit())
                ASSERT_TRUE(constraint->isValidSpanningPosition(spanning_joint_state.position));
            constraint->updateJacobians(spanning_joint_state.position);
            ASSERT_TRUE(constraint->isValidSpanningVelocity(spanning_joint_state.velocity));

            // Check that the loop constraints are properly formed
            const Eigen::VectorXd KG = constraint->K() * constraint->G();
            const Eigen::VectorXd Kg_k = constraint->K() * constraint->g() - constraint->k();
            GTEST_ASSERT_LT(KG.norm(), 1e-10)
                << "K*G: " << KG.transpose();
            GTEST_ASSERT_LT(Kg_k.norm(), 1e-10)
                << "K*g + k: " << Kg_k.transpose();

            model_state.push_back(joint_state);
            spanning_model_state.push_back(spanning_joint_state);
        }

        StatePair spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
        Eigen::VectorXd spanning_joint_pos = spanning_q_and_v.first;
        Eigen::VectorXd spanning_joint_vel = spanning_q_and_v.second;
        cluster_tree.setState(model_state);
        lgm_model.setState(spanning_joint_pos, spanning_joint_vel);

        Eigen::VectorXd pin_q(nv_span), pin_v(nv_span);
        pin_q = joint_map * spanning_joint_pos;
        pin_v = joint_map * spanning_joint_vel;

        // Extract loop constraints from the cluster tree
        cluster_tree.forwardKinematics();
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
        const Eigen::VectorXd k_pinocchio = -k_cluster;

        // Compute the forward dynamics
        Eigen::VectorXd spanning_joint_tau = Eigen::VectorXd::Random(nv_span);
        Eigen::VectorXd tau = G_cluster.transpose() * spanning_joint_tau;

        const double mu0 = 1e-14;
        Eigen::VectorXd pin_tau(nv_span);
        pin_tau = joint_map * spanning_joint_tau;

        const Eigen::VectorXd ydd_cluster = cluster_tree.forwardDynamics(tau);
        pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, mu0);
        const Eigen::VectorXd qdd_grbda = lgm_model.forwardDynamics(tau);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const Eigen::VectorXd ydd_error = qdd_grbda - (G_cluster * ydd_cluster + g_cluster);
        GTEST_ASSERT_LT(ydd_error.norm(), 1e-8)
            << "ydd_cluster: " << ydd_cluster.transpose() << "\n"
            << "G*qdd_grbda + g: " << (G_cluster * qdd_grbda + g_cluster).transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const Eigen::VectorXd qdd_error = data.ddq - joint_map * qdd_grbda;
        GTEST_ASSERT_LT(qdd_error.norm(), 1e-8)
            << "qdd_pinocchio   : " << data.ddq.transpose() << "\n"
            << "jmap * qdd_grbda: " << (joint_map * qdd_grbda).transpose();

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_grbda - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), 1e-10)
            << "K*qdd_grbda + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), 1e-10)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }
}
