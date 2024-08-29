#include "gtest/gtest.h"

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"
#include "grbda/Utils/Utilities.h"

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/rnea.hpp"
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
    test_urdf_files.push_back({urdf_directory + "double_pendulum.urdf", false});
    test_urdf_files.push_back({urdf_directory + "revolute_rotor_chain.urdf", false});
    test_urdf_files.push_back({urdf_directory + "mit_humanoid.urdf", false});
    test_urdf_files.push_back({urdf_directory + "mini_cheetah.urdf", false});
    return test_urdf_files;
}

class PinocchioExplicitNumericalValidation : public ::testing::TestWithParam<RobotSpecification>
{
};

INSTANTIATE_TEST_SUITE_P(PinocchioExplicitNumericalValidation, PinocchioExplicitNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioExplicitNumericalValidation, explicit_constraints)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    // Build models
    pinocchio::Model model;
    pinocchio::urdf::buildModel(GetParam().urdf_filename, model);
    pinocchio::Model::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam().urdf_filename, GetParam().floating_base);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
    Eigen::MatrixXd joint_map = jointMap(lgm_model, model);

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

    StatePair spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
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

class PinocchioExplicitBenchmark : public ::testing::TestWithParam<RobotSpecification>
{
};

INSTANTIATE_TEST_SUITE_P(PinocchioExplicitBenchmark, PinocchioExplicitBenchmark,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioExplicitBenchmark, compareInstructionCount)
{
    // TODO(@MatthewChignoli): Cleanup these typedefs
    typedef double Scalar;
    typedef casadi::SX ADScalar;

    typedef pinocchio::ModelTpl<Scalar> PinocchioModel;
    typedef pinocchio::ModelTpl<ADScalar> PinocchioADModel;
    typedef PinocchioADModel::Data PinocchioADData;

    using DynamicVector = Eigen::VectorXd;
    using DynamicMatrix = Eigen::MatrixXd;
    using DynamicADVector = Eigen::Vector<ADScalar, Eigen::Dynamic>;
    using DynamicADMatrix = Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic>;

    using ModelState = grbda::ModelState<ADScalar>;
    using JointState = grbda::JointState<ADScalar>;
    using StatePair = std::pair<DynamicADVector, DynamicADVector>;

    // Build models
    PinocchioModel model;
    pinocchio::urdf::buildModel(GetParam().urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADData ad_data(ad_model);

    grbda::ClusterTreeModel<ADScalar> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam().urdf_filename, GetParam().floating_base);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<ADScalar>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
    DynamicADMatrix joint_map = jointMap(lgm_model, model);

    const int nq = cluster_tree.getNumPositions();
    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    // Symbolic state
    ADScalar cs_q = ADScalar::sym("q", nq);
    ADScalar cs_v = ADScalar::sym("v", nv);

    DynamicADVector q(nq), v(nv);
    casadi::copy(cs_q, q);
    casadi::copy(cs_v, v);

    ModelState model_state;
    ModelState spanning_model_state;
    for (const auto &cluster : cluster_tree.clusters())
    {
        DynamicADVector q_i = q.segment(cluster->position_index_, cluster->num_positions_);
        DynamicADVector v_i = v.segment(cluster->velocity_index_, cluster->num_velocities_);

        JointState joint_state(grbda::JointCoordinate<ADScalar>(q, false),
                               grbda::JointCoordinate<ADScalar>(v, false));
        JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

        model_state.push_back(joint_state);
        spanning_model_state.push_back(spanning_joint_state);
    }

    StatePair spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
    DynamicADVector spanning_joint_pos = spanning_q_and_v.first;
    DynamicADVector spanning_joint_vel = spanning_q_and_v.second;
    cluster_tree.setState(model_state);
    lgm_model.setState(spanning_joint_pos, spanning_joint_vel);

    DynamicADVector pin_q(nv_span), pin_v(nv_span);
    pin_q = joint_map * spanning_joint_pos;
    pin_v = joint_map * spanning_joint_vel;

    // Extract loop constraints from the cluster tree
    // TODO(@MatthewChignoli): Sometimes we need to update kinematics before extracting G and K
    DynamicADMatrix K_cluster = DynamicADMatrix::Zero(0, 0);
    DynamicADVector k_cluster = DynamicADVector::Zero(0);
    DynamicADMatrix G_cluster = DynamicADMatrix::Zero(0, 0);
    DynamicADVector g_cluster = DynamicADVector::Zero(0);
    for (const auto &cluster : cluster_tree.clusters())
    {
        K_cluster = grbda::appendEigenMatrix(K_cluster, cluster->joint_->K());
        k_cluster = grbda::appendEigenVector(k_cluster, cluster->joint_->k());
        G_cluster = grbda::appendEigenMatrix(G_cluster, cluster->joint_->G());
        g_cluster = grbda::appendEigenVector(g_cluster, cluster->joint_->g());
    }

    // Convert implicit loop constraint to Pinocchio joint order
    const DynamicADMatrix K_pinocchio = K_cluster * joint_map.transpose();
    const DynamicADVector k_pinocchio = k_cluster;

    // Compute the forward dynamics
    ADScalar cs_tau = ADScalar::sym("tau", nv_span);
    DynamicADVector spanning_joint_tau(nv_span);
    casadi::copy(cs_tau, spanning_joint_tau);
    DynamicADVector tau = G_cluster.transpose() * spanning_joint_tau;

    const ADScalar mu0 = 1e-12;
    DynamicADVector pin_tau(nv_span);
    pin_tau = joint_map * spanning_joint_tau;

    const DynamicADVector ydd_cluster = cluster_tree.forwardDynamics(tau);
    pinocchio::forwardDynamics(ad_model, ad_data, pin_q, pin_v, pin_tau, K_pinocchio, k_pinocchio, mu0);
    const DynamicADVector qdd_grbda = lgm_model.forwardDynamics(tau);

    // Create casadi function to compute forward dynamics
    ADScalar cs_ydd_cluster = ADScalar(casadi::Sparsity::dense(nv, 1));
    casadi::copy(ydd_cluster, cs_ydd_cluster);

    ADScalar cs_qdd_grbda = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_grbda, cs_qdd_grbda);

    ADScalar cs_qdd_pinocchio = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(ad_data.ddq, cs_qdd_pinocchio);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_ydd_cluster});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio});

    casadi::Function csGrbdaFD("grbdaFD",
                               casadi::SXVector{cs_q, cs_v, cs_tau},
                               casadi::SXVector{cs_qdd_grbda});

    // print the numbers of insructions
    std::cout << "cABA instructions: " << csClusterABA.n_instructions() << std::endl;
    std::cout << "pinocchioFD instructions: " << csPinocchioFD.n_instructions() << std::endl;
    std::cout << "grbdaFD instructions: " << csGrbdaFD.n_instructions() << std::endl;

    // Check these against each other
    // TODO(@MatthewChignoli): Need better naming for these tests
    for (int i = 0; i < 40; ++i)
    {
        std::vector<casadi::DM> dm_q = grbda::random<casadi::DM>(nq);
        std::vector<casadi::DM> dm_v = grbda::random<casadi::DM>(nv);
        std::vector<casadi::DM> dm_tau = grbda::random<casadi::DM>(nv_span);
        casadi::DMVector dm_res1 = csClusterABA(casadi::DMVector{dm_q, dm_v, dm_tau});
        casadi::DMVector dm_res2 = csPinocchioFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        casadi::DMVector dm_res3 = csGrbdaFD(casadi::DMVector{dm_q, dm_v, dm_tau});

        DynamicVector res1(nv), res2(nv_span), res3(nv_span);
        casadi::copy(dm_res1[0], res1);
        casadi::copy(dm_res2[0], res2);
        casadi::copy(dm_res3[0], res3);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const DynamicMatrix G_double = G_cluster.cast<double>();
        const DynamicVector g_double = g_cluster.cast<double>();
        const DynamicVector ydd_error = res3 - (G_double * res1 + g_double);
        GTEST_ASSERT_LT(ydd_error.norm(), 1e-10)
            << "qdd_grbda: " << res3.transpose() << "\n"
            << "G*ydd_grbda + g: " << (G_double * res1 + g_double).transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DynamicVector qdd_error = res2 - joint_map.cast<double>() * res3;
        GTEST_ASSERT_LT(qdd_error.norm(), 1e-10)
            << "qdd_pinocchio   : " << res2.transpose() << "\n"
            << "jmap * qdd_grbda: " << (joint_map.cast<double>() * res3).transpose();
    }
}
