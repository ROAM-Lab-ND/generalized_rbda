#include <iostream>
#include <fstream>

#include "gtest/gtest.h"

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"
#include "grbda/Utils/Utilities.h"
#include "grbda/Utils/Timer.h"

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

const std::string path_to_data = SOURCE_DIRECTORY "/Benchmarking/data/TimingPinocchioFD_";

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
public:
    std::ofstream outfile;
    grbda::Timer timer;
    const int num_samples = 25;

private:
    void SetUp() override {
        outfile.open(path_to_data + "NumericalValidation.csv", std::ios::app);
        if (!outfile.is_open()) {
            std::cerr << "Failed to open file." << std::endl;
        }
    }

    void TearDown() override {
        if (outfile.is_open()) {
            outfile.close();
        }
    }
};

INSTANTIATE_TEST_SUITE_P(PinocchioNumericalValidation, PinocchioNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioNumericalValidation, forward_dynamics)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    double t_cluster = 0.;
    double t_pinocchio = 0.;
    double t_lg = 0.;
    
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

    for (int i = 0; i < num_samples; i++)
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
            // Check that the loop constraints are properly formed
            std::shared_ptr<LoopConstraint> constraint = cluster->joint_->cloneLoopConstraint();
            const Eigen::VectorXd KG = constraint->K() * constraint->G();
            const Eigen::VectorXd Kg_k = constraint->K() * constraint->g() - constraint->k();
            GTEST_ASSERT_GT(constraint->K().norm(), 1e-10);
            GTEST_ASSERT_GT(constraint->G().norm(), 1e-10);
            GTEST_ASSERT_LT(KG.norm(), 1e-10)
                << "K*G: " << KG.transpose();
            GTEST_ASSERT_LT(Kg_k.norm(), 1e-10)
                << "K*g - k: " << Kg_k.transpose();

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

        timer.start();
        const Eigen::VectorXd ydd_cluster = cluster_tree.forwardDynamics(tau);
        t_cluster += timer.getMs();

        timer.start();
        pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, mu0);
        t_pinocchio += timer.getMs();

        timer.start();
        const Eigen::VectorXd qdd_grbda = lgm_model.forwardDynamics(tau);
        t_lg += timer.getMs();

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

    outfile << GetParam().urdf_filename << ","
            << t_cluster / num_samples << ","
            << t_pinocchio / num_samples << ","
            << t_lg / num_samples << std::endl;
}

class PinocchioBenchmark : public ::testing::TestWithParam<RobotSpecification>
{
public:
    std::ofstream outfile;
    grbda::Timer timer;
    const int num_samples = 25;

private:
    void SetUp() override {
        outfile.open(path_to_data + "InstructionCount.csv", std::ios::app);
        if (!outfile.is_open()) {
            std::cerr << "Failed to open file." << std::endl;
        }
    }

    void TearDown() override {
        if (outfile.is_open()) {
            outfile.close();
        }
    }
};

INSTANTIATE_TEST_SUITE_P(PinocchioBenchmark, PinocchioBenchmark,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioBenchmark, compareInstructionCount)
{
    using Scalar = double;
    using ADScalar = casadi::SX;

    using PinocchioModel = pinocchio::ModelTpl<Scalar>;
    using PinocchioADModel = pinocchio::ModelTpl<ADScalar>;

    using DynamicVector = Eigen::VectorXd;
    using DynamicMatrix = Eigen::MatrixXd;
    using DynamicADVector = Eigen::Vector<ADScalar, Eigen::Dynamic>;
    using DynamicADMatrix = Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic>;

    using ModelState = grbda::ModelState<ADScalar>;
    using JointCoordinate = grbda::JointCoordinate<ADScalar>;
    using JointState = grbda::JointState<ADScalar>;
    using StatePair = std::pair<DynamicADVector, DynamicADVector>;
    using LoopConstraintPtr = std::shared_ptr<grbda::LoopConstraint::Base<ADScalar>>;

    double t_cluster = 0.;
    double t_pinocchio = 0.;
    double t_lg = 0.;

    // Build models
    PinocchioModel model;
    pinocchio::urdf::buildModel(GetParam().urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADModel::Data ad_data(ad_model);

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

        JointState joint_state;
        LoopConstraintPtr loop_constraint = cluster->joint_->cloneLoopConstraint();
        joint_state.position = JointCoordinate(q_i, !loop_constraint->isExplicit());
        joint_state.velocity = JointCoordinate(v_i, false);

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
    cluster_tree.forwardKinematics();
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
    const DynamicADVector k_pinocchio = -k_cluster;

    // Compute the forward dynamics
    ADScalar cs_tau = ADScalar::sym("tau", nv_span);
    DynamicADVector spanning_joint_tau(nv_span);
    casadi::copy(cs_tau, spanning_joint_tau);
    DynamicADVector tau = G_cluster.transpose() * spanning_joint_tau;

    const ADScalar mu0 = 1e-14;
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

    ADScalar cs_G = ADScalar(casadi::Sparsity::dense(G_cluster.rows(), G_cluster.cols()));
    casadi::copy(G_cluster, cs_G);

    ADScalar cs_g = ADScalar(casadi::Sparsity::dense(g_cluster.rows(), 1));
    casadi::copy(g_cluster, cs_g);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_ydd_cluster});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio});

    casadi::Function csGrbdaFD("grbdaFD",
                               casadi::SXVector{cs_q, cs_v, cs_tau},
                               casadi::SXVector{cs_qdd_grbda});

    casadi::Function csExplicitJacobian("explicitJacobian",
                                        casadi::SXVector{cs_q, cs_v},
                                        casadi::SXVector{cs_G, cs_g});

    // print the numbers of insructions
    std::cout << "cABA instructions: " << csClusterABA.n_instructions() << std::endl;
    std::cout << "pinocchioFD instructions: " << csPinocchioFD.n_instructions() << std::endl;
    std::cout << "grbdaFD instructions: " << csGrbdaFD.n_instructions() << std::endl;

    // Check the solutions against each other
    using ScalarModel = grbda::ClusterTreeModel<Scalar>;
    using ScalarRigidBodyTreeModel = grbda::RigidBodyTreeModel<Scalar>;
    using ScalarModelState = grbda::ModelState<Scalar>;
    using ScalarJointState = grbda::JointState<Scalar>;
    using ScalarStatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
    for (int i = 0; i < num_samples; ++i)
    {
        ScalarModel numerical_cluster_tree;
        numerical_cluster_tree.buildModelFromURDF(GetParam().urdf_filename,
                                                  GetParam().floating_base);

        ScalarModelState scalar_model_state;
        for (const auto &cluster : numerical_cluster_tree.clusters())
        {
            ScalarJointState joint_state = cluster->joint_->randomJointState();
            ScalarJointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);
            scalar_model_state.push_back(joint_state);
        }
        numerical_cluster_tree.setState(scalar_model_state);
        numerical_cluster_tree.forwardKinematics();

        ScalarStatePair scalar_q_and_v = grbda::modelStateToVector(scalar_model_state);
        DynamicVector scalar_q = scalar_q_and_v.first;
        casadi::DMVector dm_q;
        for (int j = 0; j < scalar_q.size(); ++j)
        {
            dm_q.push_back(scalar_q(j));
        }
        casadi::DMVector dm_v = grbda::random<casadi::DM>(nv);
        casadi::DMVector dm_tau = grbda::random<casadi::DM>(nv_span);

        timer.start();
        casadi::DMVector dm_cABA_res = csClusterABA(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_cluster += timer.getMs();

        timer.start();
        casadi::DMVector dm_pin_res = csPinocchioFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pinocchio += timer.getMs();

        timer.start();
        casadi::DMVector dm_lgm_res = csGrbdaFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_lg += timer.getMs();

        casadi::DMVector dm_explicit = csExplicitJacobian(casadi::DMVector{dm_q, dm_v});

        DynamicVector cABA_res(nv), pin_res(nv_span), lgm_res(nv_span);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_pin_res[0], pin_res);
        casadi::copy(dm_lgm_res[0], lgm_res);

        DynamicMatrix G_res(nv_span, nv);
        casadi::copy(dm_explicit[0], G_res);
        DynamicVector g_res(nv_span);
        casadi::copy(dm_explicit[1], g_res);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        DynamicVector qdd_cABA = G_res * cABA_res + g_res;
        const DynamicVector grbda_error = lgm_res - qdd_cABA;
        GTEST_ASSERT_LT(grbda_error.norm(), 1e-8)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << qdd_cABA.transpose() << "\n"
            << "cABA_res: " << cABA_res.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DynamicVector qdd_error = pin_res - joint_map.cast<Scalar>() * lgm_res;
        GTEST_ASSERT_LT(qdd_error.norm(), 1e-8)
            << "qdd_pinocchio   : " << pin_res.transpose() << "\n"
            << "jmap * qdd_grbda: " << (joint_map.cast<Scalar>() * lgm_res).transpose();
    }

    outfile << GetParam().urdf_filename << ","
            << t_cluster / num_samples << ","
            << t_pinocchio / num_samples << ","
            << t_lg / num_samples << std::endl;
}