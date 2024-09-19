#include <regex>
#include "pinocchioHelpers.hpp"
#include "grbda/Robots/RobotTypes.h"

template <typename Scalar>
struct RobotSpecification
{
    std::string urdf_filename;
    std::string approx_urdf_filename;
    grbda::ClusterTreeModel<Scalar> cluster_tree;
    bool floating_base; // TODO(@MatthewChignoli): I think we are assuming no floating bases so can remove...
    
    std::string outfile_suffix;
    std::string instruction_prefix = "InstructionPinocchioFD_";
    std::string timing_prefix = "TimingPinocchioFD_";
    std::ofstream instruction_outfile;
    std::ofstream timing_outfile;
    std::string name;

    RobotSpecification(grbda::ClusterTreeModel<Scalar> cluster_tree_,
                       std::string urdf_filename_, bool floating_base_,
                       std::string outfile_suffix_ = "Approx")
        : cluster_tree(cluster_tree_),
          urdf_filename(urdf_filename_ + ".urdf"),
          approx_urdf_filename(urdf_filename_ + "_approximate.urdf"),
          floating_base(floating_base_), outfile_suffix(outfile_suffix_)
    {
        registerNameFromUrdfFilename(urdf_filename);
        openOutfile();
    }

    ~RobotSpecification()
    {
        closeOutfile();
    }

    void registerNameFromUrdfFilename(const std::string &url)
    {
        // Find the last occurrence of '/' which indicates the start of the filename
        size_t lastSlashPos = url.find_last_of('/');
        if (lastSlashPos == std::string::npos)
        {
            lastSlashPos = -1;
        }

        // Extract filename
        std::string filename = url.substr(lastSlashPos + 1);

        // Find the last occurrence of '.' which indicates the start of the file extension
        size_t dotPos = filename.find_last_of('.');
        if (dotPos != std::string::npos)
        {
            // Subtract extension
            filename = filename.substr(0, dotPos);
        }

        name = filename;
    }

    void openOutfile()
    {
        instruction_outfile.open(path_to_data + instruction_prefix + outfile_suffix + ".csv", std::ios::app);
        if (!instruction_outfile.is_open())
        {
            std::cerr << "Failed to open instruction benchmark file." << std::endl;
        }
        timing_outfile.open(path_to_data + timing_prefix + outfile_suffix + ".csv", std::ios::app);
        if (!timing_outfile.is_open())
        {
            std::cerr << "Failed to open time benchmark file." << std::endl;
        }
    }

    void closeOutfile()
    {
        if (instruction_outfile.is_open())
        {
            instruction_outfile.close();
        }

        if (timing_outfile.is_open())
        {
            timing_outfile.close();
        }
    }

    virtual void writeToFile(std::ofstream &outfile,
                             double i_cluster, double i_approx, double i_pinocchio)
    {
        outfile << name << ","
                << i_cluster << ","
                << i_approx << ","
                << i_pinocchio << std::endl;
    }
};

void clearOldDataFiles()
{
    for (std::string bench_type : {"Instruction", "Timing"})
    {
        std::ofstream outfile;
        outfile.open(path_to_data + bench_type + "PinocchioFD_Approx.csv", std::ios::trunc);
        if (!outfile.is_open())
        {
            std::cerr << "Failed to open file." << std::endl;
        }
        if (outfile.is_open())
        {
            outfile.close();
        }
    }
}

template <typename Scalar>
using SpecVector = std::vector<std::shared_ptr<RobotSpecification<Scalar>>>;

SpecVector<double> RobotSpecificationAsDouble()
{
    clearOldDataFiles();

    // TODO(@MatthewChignoli): These should be floating base...
    SpecVector<double> urdf_files;
    
    {
        std::string urdf_file = main_urdf_directory + "mini_cheetah";
        grbda::ClusterTreeModel<double> mini_cheetah_cluster_tree;
        mini_cheetah_cluster_tree.buildModelFromURDF(urdf_file + ".urdf", false);
        urdf_files.push_back(std::make_shared<RobotSpecification<double>>(
            mini_cheetah_cluster_tree, urdf_file, false));
    }

    {
        std::string urdf_file = main_urdf_directory + "mit_humanoid";
        grbda::ClusterTreeModel<double> mit_humanoid_cluster_tree;
        mit_humanoid_cluster_tree.buildModelFromURDF(urdf_file + ".urdf", false);
        urdf_files.push_back(std::make_shared<RobotSpecification<double>>(
            mit_humanoid_cluster_tree, urdf_file, false));
    }
    
    {
        std::string urdf_file = main_urdf_directory + "tello_humanoid";
        grbda::TelloWithArms robot;
        grbda::ClusterTreeModel<double> tello_cluster_tree(robot.buildClusterTreeModel());
        urdf_files.push_back(std::make_shared<RobotSpecification<double>>(
            tello_cluster_tree, urdf_file, false));
    }

    {
        std::string urdf_file = main_urdf_directory + "jvrc1_humanoid";
        grbda::ClusterTreeModel<double> jvrc1_cluster_tree;
        jvrc1_cluster_tree.buildModelFromURDF(urdf_file + ".urdf", false);
        urdf_files.push_back(std::make_shared<RobotSpecification<double>>(
            jvrc1_cluster_tree, urdf_file, false));
    }

    return urdf_files;
}

class PinocchioNumericalValidation : public ::testing::TestWithParam<std::shared_ptr<RobotSpecification<double>>>
{
public:
    grbda::Timer timer;
    const int num_samples = 25;
};

INSTANTIATE_TEST_SUITE_P(PinocchioNumericalValidation, PinocchioNumericalValidation,
                         ::testing::ValuesIn(RobotSpecificationAsDouble()));

TEST_P(PinocchioNumericalValidation, forward_dynamics)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    double t_cluster = 0.;
    double t_approx = 0.;
    double t_lg = 0.;

    // Build models
    grbda::ClusterTreeModel<double>& cluster_tree = GetParam()->cluster_tree;
    grbda::ClusterTreeModel<double> approx_tree;
    approx_tree.buildModelFromURDF(GetParam()->approx_urdf_filename, GetParam()->floating_base);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);

    // TODO(@MatthewChignoli): Commented this out for Tello because no independent position for cluster tree model
    // GTEST_ASSERT_EQ(cluster_tree.getNumPositions(), approx_tree.getNumPositions());
    GTEST_ASSERT_EQ(cluster_tree.getNumDegreesOfFreedom(), approx_tree.getNumDegreesOfFreedom());

    const int nv = cluster_tree.getNumDegreesOfFreedom();
    // const int nv_span = lgm_model.getNumDegreesOfFreedom();

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
        approx_tree.setState(grbda::modelStateToVector(model_state));
        lgm_model.setState(spanning_joint_pos, spanning_joint_vel);

        // // Extract loop constraints from the cluster tree
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
            GTEST_ASSERT_LT(KG.norm(), 1e-10) << "K*G: " << KG.transpose();
            GTEST_ASSERT_LT(Kg_k.norm(), 1e-10) << "K*g - k: " << Kg_k.transpose();

            K_cluster = grbda::appendEigenMatrix(K_cluster, cluster->joint_->K());
            k_cluster = grbda::appendEigenVector(k_cluster, cluster->joint_->k());
            G_cluster = grbda::appendEigenMatrix(G_cluster, cluster->joint_->G());
            g_cluster = grbda::appendEigenVector(g_cluster, cluster->joint_->g());
        }
        GTEST_ASSERT_GT(K_cluster.norm(), 1e-10);
        GTEST_ASSERT_GT(G_cluster.norm(), 1e-10);

        // Compute the forward dynamics
        Eigen::VectorXd tau = Eigen::VectorXd::Random(nv);

        timer.start();
        const Eigen::VectorXd ydd_cluster = cluster_tree.forwardDynamics(tau);
        Eigen::VectorXd qdd_cluster = Eigen::VectorXd::Zero(0);
        for (const auto &cluster : cluster_tree.clusters())
        {
            Eigen::VectorXd ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                        cluster->num_velocities_);
            Eigen::VectorXd qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
            qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
        }
        t_cluster += timer.getMs();

        timer.start();
        const Eigen::VectorXd ydd_approx = approx_tree.forwardDynamics(tau);
        t_approx += timer.getMs();

        timer.start();
        const Eigen::VectorXd qdd_lgm = lgm_model.forwardDynamics(tau);
        t_lg += timer.getMs();

        // Check grbda cluster tree solution against lagrange multiplier solution
        const Eigen::VectorXd grbda_error = qdd_lgm - qdd_cluster;
        GTEST_ASSERT_LT(grbda_error.norm(), 1e-6)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // TODO(@MatthewChignoli): Is there any way to compare cluster tree against approx tree?

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;
        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), 1e-10)
            << "K*qdd_grbda + k    : " << cnstr_violation_grbda.transpose();
    }
}

SpecVector<casadi::SX> RobotSpecificationAsSX()
{
    clearOldDataFiles();

    // TODO(@MatthewChignoli): These should be floating base...
    SpecVector<casadi::SX> urdf_files;
    
    {
        std::string urdf_file = main_urdf_directory + "mini_cheetah";
        grbda::ClusterTreeModel<casadi::SX> mini_cheetah_cluster_tree;
        mini_cheetah_cluster_tree.buildModelFromURDF(urdf_file + ".urdf", false);
        urdf_files.push_back(std::make_shared<RobotSpecification<casadi::SX>>(
            mini_cheetah_cluster_tree, urdf_file, false));
    }

    {
        std::string urdf_file = main_urdf_directory + "mit_humanoid";
        grbda::ClusterTreeModel<casadi::SX> mit_humanoid_cluster_tree;
        mit_humanoid_cluster_tree.buildModelFromURDF(urdf_file + ".urdf", false);
        urdf_files.push_back(std::make_shared<RobotSpecification<casadi::SX>>(
            mit_humanoid_cluster_tree, urdf_file, false));
    }

    // {
    //     std::string urdf_file = main_urdf_directory + "tello";
    //     grbda::Tello robot;
    //     grbda::ClusterTreeModel<casadi::SX> tello_cluster_tree(robot.buildClusterTreeModel());
    // }

    {
        std::string urdf_file = main_urdf_directory + "jvrc1_humanoid";
        grbda::ClusterTreeModel<casadi::SX> jvrc1_cluster_tree;
        jvrc1_cluster_tree.buildModelFromURDF(urdf_file + ".urdf", false);
        urdf_files.push_back(std::make_shared<RobotSpecification<casadi::SX>>(
            jvrc1_cluster_tree, urdf_file, false));
    }

    return urdf_files;
}

class PinocchioBenchmark
    : public ::testing::TestWithParam<std::shared_ptr<RobotSpecification<casadi::SX>>>
{
public:
    grbda::Timer timer;
    const int num_samples = 25;
};

INSTANTIATE_TEST_SUITE_P(PinocchioBenchmark, PinocchioBenchmark,
                         ::testing::ValuesIn(RobotSpecificationAsSX()));

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
    double t_approx = 0.;
    double t_pinocchio = 0.;
    double t_lg = 0.;

    // Build models
    PinocchioModel model;
    pinocchio::urdf::buildModel(GetParam()->urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADModel::Data ad_data(ad_model);

    grbda::ClusterTreeModel<ADScalar>& cluster_tree = GetParam()->cluster_tree;
    grbda::ClusterTreeModel<ADScalar> approx_tree;
    approx_tree.buildModelFromURDF(GetParam()->approx_urdf_filename, GetParam()->floating_base);

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
    approx_tree.setState(grbda::modelStateToVector(model_state));
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
    const DynamicADVector ydd_approx = approx_tree.forwardDynamics(tau);
    pinocchio::forwardDynamics(ad_model, ad_data, pin_q, pin_v, pin_tau, K_pinocchio, k_pinocchio, mu0);
    const DynamicADVector qdd_grbda = lgm_model.forwardDynamics(tau);

    // Create casadi function to compute forward dynamics
    ADScalar cs_ydd_cluster = ADScalar(casadi::Sparsity::dense(nv, 1));
    casadi::copy(ydd_cluster, cs_ydd_cluster);

    ADScalar cs_ydd_approx = ADScalar(casadi::Sparsity::dense(nv, 1));
    casadi::copy(ydd_approx, cs_ydd_approx);

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

    casadi::Function csApproxABA("clusterApprox",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_ydd_approx});

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
    std::cout << "aABA instructions: " << csApproxABA.n_instructions() << std::endl;
    std::cout << "pinocchioFD instructions: " << csPinocchioFD.n_instructions() << std::endl;
    std::cout << "grbdaFD instructions: " << csGrbdaFD.n_instructions() << std::endl;
    const int i_cluster = static_cast<int>(csClusterABA.n_instructions());
    const int i_approx = static_cast<int>(csApproxABA.n_instructions());
    const int i_pinocchio = static_cast<int>(csPinocchioFD.n_instructions());
    const int i_lg = static_cast<int>(csGrbdaFD.n_instructions());

    // Check the solutions against each other
    using ScalarModel = grbda::ClusterTreeModel<Scalar>;
    using ScalarRigidBodyTreeModel = grbda::RigidBodyTreeModel<Scalar>;
    using ScalarModelState = grbda::ModelState<Scalar>;
    using ScalarJointState = grbda::JointState<Scalar>;
    using ScalarStatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
    for (int i = 0; i < num_samples; ++i)
    {
        ScalarModel numerical_cluster_tree;
        numerical_cluster_tree.buildModelFromURDF(GetParam()->urdf_filename,
                                                  GetParam()->floating_base);

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
        casadi::DMVector dm_aABA_res = csApproxABA(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_approx += timer.getMs();

        timer.start();
        casadi::DMVector dm_pin_res = csPinocchioFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pinocchio += timer.getMs();

        timer.start();
        casadi::DMVector dm_lgm_res = csGrbdaFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_lg += timer.getMs();

        casadi::DMVector dm_explicit = csExplicitJacobian(casadi::DMVector{dm_q, dm_v});

        DynamicVector cABA_res(nv), aABA_res(nv), pin_res(nv_span), lgm_res(nv_span);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_aABA_res[0], aABA_res);
        casadi::copy(dm_pin_res[0], pin_res);
        casadi::copy(dm_lgm_res[0], lgm_res);

        DynamicMatrix G_res(nv_span, nv);
        casadi::copy(dm_explicit[0], G_res);
        DynamicVector g_res(nv_span);
        casadi::copy(dm_explicit[1], g_res);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        DynamicVector qdd_cABA = G_res * cABA_res + g_res;
        const DynamicVector grbda_error = lgm_res - qdd_cABA;
        GTEST_ASSERT_LT(grbda_error.norm(), 1e-6)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << qdd_cABA.transpose() << "\n"
            << "cABA_res: " << cABA_res.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DynamicVector qdd_error = pin_res - joint_map.cast<Scalar>() * lgm_res;
        GTEST_ASSERT_LT(qdd_error.norm(), 1e-6)
            << "qdd_pinocchio   : " << pin_res.transpose() << "\n"
            << "jmap * qdd_grbda: " << (joint_map.cast<Scalar>() * lgm_res).transpose();
    }

    GetParam()->writeToFile(GetParam()->instruction_outfile,
                            i_cluster,i_approx, i_pinocchio);

    GetParam()->writeToFile(GetParam()->timing_outfile,
                            t_cluster / num_samples,
                            t_approx / num_samples,
                            t_pinocchio / num_samples);
}
