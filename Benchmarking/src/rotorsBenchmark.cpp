#include <regex>
#include "pinocchioHelpers.hpp"

// TODO(@MatthewChignoli): Change name of this file to "branch and depth benchmark"
struct RobotSpecification
{
    std::string urdf_filename;
    std::string outfile_suffix;
    std::string instruction_prefix = "InstructionPinocchioFD_";
    std::string timing_prefix = "TimingPinocchioFD_";
    std::ofstream instruction_outfile;
    std::ofstream timing_outfile;
    std::string name;

    RobotSpecification(std::string urdf_filename, std::string outfile_suffix = "systems")
        : urdf_filename(urdf_filename), outfile_suffix(outfile_suffix)
    {
        registerNameFromUrdfFilename(urdf_filename);
        openOutfile();
    }

    virtual ~RobotSpecification()
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

    virtual void writeToFile(std::ofstream &outfile, double i_cluster, double i_pinocchio, double i_lg)
    {
        outfile << name << ","
                << i_cluster << ","
                << i_pinocchio << ","
                << i_lg << std::endl;
    }
};

struct BranchAndDepthSpecification : public RobotSpecification
{
    int branch_count;
    int depth_count;

    BranchAndDepthSpecification(std::string urdf_filename,
                                std::string outfile_suffix = "revolute_chain")
        : RobotSpecification(urdf_filename, outfile_suffix)
    {
        registerBranchAndDepthCountFromName(name);
    }

    void registerBranchAndDepthCountFromName(const std::string &name)
    {
        std::regex re(R"(_(\d+)_+(\d+)_?)");
        std::smatch match;

        if (std::regex_search(name, match, re) && match.size() > 2)
        {
            branch_count = std::stoi(match.str(1));
            depth_count = std::stoi(match.str(2));
        }
    }

    void writeToFile(std::ofstream &outfile, double i_cluster, double i_pinocchio, double i_lg) override
    {
        outfile << branch_count << ","
                << depth_count << ","
                << i_cluster << ","
                << i_pinocchio << ","
                << i_lg << std::endl;
    }
};

using SpecVector = std::vector<std::shared_ptr<RobotSpecification>>;

SpecVector GetIndividualUrdfFiles()
{

    SpecVector urdf_files;
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "four_bar.urdf"));
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "revolute_rotor_chain.urdf"));
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "mit_humanoid.urdf"));
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "mini_cheetah.urdf"));
    return urdf_files;
}

SpecVector GetRevoluteRotorUrdfFiles()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_revolute_urdf dir
    SpecVector urdf_files;
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7, 8, 9, 10})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_revolute_urdf/revolute_rotor_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            urdf_files.push_back(std::make_shared<BranchAndDepthSpecification>(urdf_file));
        }
    return urdf_files;
}

SpecVector GetRevoluteRotorPairUrdfFiles()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_revolute_pair_urdf dir
    SpecVector urdf_files;
    std::string outfile_suffix = "revolute_pair_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_revolute_pair_urdf/revolute_rotor_pair_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            urdf_files.push_back(std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                               outfile_suffix));
        }
    return urdf_files;
}

SpecVector GetFourBarUrdfFiles()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_four_bar_urdf dir
    SpecVector urdf_files;
    std::string outfile_suffix = "four_bar_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_four_bar_urdf/four_bar_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            urdf_files.push_back(std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                               outfile_suffix));
        }
    return urdf_files;
}

SpecVector GetBenchmarkUrdfFiles()
{
    std::vector<std::string> sys_types = {"revolute_chain", "revolute_pair_chain",
                                          "four_bar_chain", "systems"};
    for (std::string bench_type : {"Instruction", "Timing"})
    {
        for (std::string sys_type : sys_types)
        {
            std::ofstream outfile;
            outfile.open(path_to_data + bench_type + "PinocchioFD_" + sys_type + ".csv",
                         std::ios::trunc);
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

    SpecVector test_urdf_files = GetIndividualUrdfFiles();

    SpecVector revrotor_urdf_files = GetRevoluteRotorUrdfFiles();
    test_urdf_files.insert(test_urdf_files.end(), revrotor_urdf_files.begin(),
                           revrotor_urdf_files.end());

    SpecVector revrotor_pair_urdf_files = GetRevoluteRotorPairUrdfFiles();
    test_urdf_files.insert(test_urdf_files.end(), revrotor_pair_urdf_files.begin(),
                           revrotor_pair_urdf_files.end());

    SpecVector four_bar_urdf_files = GetFourBarUrdfFiles();
    test_urdf_files.insert(test_urdf_files.end(), four_bar_urdf_files.begin(),
                           four_bar_urdf_files.end());

    return test_urdf_files;
}

class PinocchioNumericalValidation : public ::testing::TestWithParam<std::shared_ptr<RobotSpecification>>
{
public:
    const int num_samples = 8;
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
    pinocchio::urdf::buildModel(GetParam()->urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam()->urdf_filename);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
    JointMap<double> joint_map = jointMap(lgm_model, model);

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
        pin_q = joint_map.pos * spanning_joint_pos;
        pin_v = joint_map.vel* spanning_joint_vel;

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
            GTEST_ASSERT_LT(KG.norm(), 1e-10)
                << "K*G: " << KG.transpose();
            GTEST_ASSERT_LT(Kg_k.norm(), 1e-10)
                << "K*g - k: " << Kg_k.transpose();

            K_cluster = grbda::appendEigenMatrix(K_cluster, cluster->joint_->K());
            k_cluster = grbda::appendEigenVector(k_cluster, cluster->joint_->k());
            G_cluster = grbda::appendEigenMatrix(G_cluster, cluster->joint_->G());
            g_cluster = grbda::appendEigenVector(g_cluster, cluster->joint_->g());
        }
        GTEST_ASSERT_GT(K_cluster.norm(), 1e-10);
        GTEST_ASSERT_GT(G_cluster.norm(), 1e-10);

        // Convert implicit loop constraint to Pinocchio joint order
        const Eigen::MatrixXd K_pinocchio = K_cluster * joint_map.vel.transpose();
        const Eigen::VectorXd k_pinocchio = -k_cluster;

        // Compute the forward dynamics
        Eigen::VectorXd spanning_joint_tau = Eigen::VectorXd::Random(nv_span);
        Eigen::VectorXd tau = G_cluster.transpose() * spanning_joint_tau;

        const double mu0 = 1e-14;
        Eigen::VectorXd pin_tau(nv_span);
        pin_tau = joint_map.vel* spanning_joint_tau;

        const Eigen::VectorXd ydd_cluster = cluster_tree.forwardDynamics(tau);
        Eigen::VectorXd qdd_cluster = Eigen::VectorXd::Zero(0);
        for (const auto &cluster : cluster_tree.clusters())
        {
            Eigen::VectorXd ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                        cluster->num_velocities_);
            Eigen::VectorXd qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
            qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
        }

        pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, mu0);

        const Eigen::VectorXd qdd_lgm = lgm_model.forwardDynamics(tau);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const Eigen::VectorXd grbda_error = qdd_lgm - qdd_cluster;
        GTEST_ASSERT_LT(grbda_error.norm(), 1e-6)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const Eigen::VectorXd qdd_error = data.ddq - joint_map.vel* qdd_lgm;
        GTEST_ASSERT_LT(qdd_error.norm(), 1e-6)
            << "qdd_pinocchio   : " << data.ddq.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.vel* qdd_lgm).transpose();

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), 1e-10)
            << "K*qdd_lgm + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), 1e-10)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }
}

class PinocchioBenchmark : public ::testing::TestWithParam<std::shared_ptr<RobotSpecification>>
{
public:
    grbda::Timer timer;
    const int num_samples = 25;
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
    pinocchio::urdf::buildModel(GetParam()->urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADModel::Data ad_data(ad_model);

    grbda::ClusterTreeModel<ADScalar> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam()->urdf_filename);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<ADScalar>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
    JointMap<ADScalar> joint_map = jointMap(lgm_model, model);

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
    pin_q = joint_map.pos * spanning_joint_pos;
    pin_v = joint_map.vel* spanning_joint_vel;

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
    const DynamicADMatrix K_pinocchio = K_cluster * joint_map.vel.transpose();
    const DynamicADVector k_pinocchio = -k_cluster;

    // Compute the forward dynamics
    ADScalar cs_tau = ADScalar::sym("tau", nv_span);
    DynamicADVector spanning_joint_tau(nv_span);
    casadi::copy(cs_tau, spanning_joint_tau);
    DynamicADVector tau = G_cluster.transpose() * spanning_joint_tau;

    const ADScalar mu0 = 1e-14;
    DynamicADVector pin_tau(nv_span);
    pin_tau = joint_map.vel* spanning_joint_tau;

    const DynamicADVector ydd_cluster = cluster_tree.forwardDynamics(tau);
    DynamicADVector qdd_cluster = DynamicADVector::Zero(0);
    for (const auto &cluster : cluster_tree.clusters())
    {
        DynamicADVector ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                    cluster->num_velocities_);
        DynamicADVector qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();  
        qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
    }

    pinocchio::forwardDynamics(ad_model, ad_data, pin_q, pin_v, pin_tau, K_pinocchio, k_pinocchio, mu0);
    
    const DynamicADVector qdd_grbda = lgm_model.forwardDynamics(tau);

    // Create casadi function to compute forward dynamics
    ADScalar cs_qdd_cluster = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_cluster, cs_qdd_cluster);

    ADScalar cs_qdd_grbda = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_grbda, cs_qdd_grbda);

    ADScalar cs_qdd_pinocchio = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(ad_data.ddq, cs_qdd_pinocchio);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_qdd_cluster});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio});

    casadi::Function csGrbdaFD("grbdaFD",
                               casadi::SXVector{cs_q, cs_v, cs_tau},
                               casadi::SXVector{cs_qdd_grbda});

    const int i_cluster = static_cast<int>(csClusterABA.n_instructions());
    const int i_pinocchio = static_cast<int>(csPinocchioFD.n_instructions());
    const int i_lg = static_cast<int>(csGrbdaFD.n_instructions());

    // Check the solutions against each other
    using ScalarModel = grbda::ClusterTreeModel<Scalar>;
    using ScalarModelState = grbda::ModelState<Scalar>;
    using ScalarJointState = grbda::JointState<Scalar>;
    using ScalarStatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
    for (int i = 0; i < num_samples; ++i)
    {
        ScalarModel numerical_cluster_tree;
        numerical_cluster_tree.buildModelFromURDF(GetParam()->urdf_filename);

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

        DynamicVector cABA_res(nv_span), pin_res(nv_span), lgm_res(nv_span);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_pin_res[0], pin_res);
        casadi::copy(dm_lgm_res[0], lgm_res);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        const DynamicVector grbda_error = lgm_res - cABA_res;
        GTEST_ASSERT_LT(grbda_error.norm(), 1e-4)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << cABA_res.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DynamicVector qdd_error = pin_res - joint_map.vel.cast<Scalar>() * lgm_res;
        GTEST_ASSERT_LT(qdd_error.norm(), 1e-4)
            << "qdd_pinocchio   : " << pin_res.transpose() << "\n"
            << "jmap * qdd_grbda: " << (joint_map.vel.cast<Scalar>() * lgm_res).transpose();
    }

    GetParam()->writeToFile(GetParam()->instruction_outfile,
                            i_cluster, i_pinocchio, i_lg);

    GetParam()->writeToFile(GetParam()->timing_outfile,
                            t_cluster / num_samples,
                            t_pinocchio / num_samples,
                            t_lg / num_samples);
}
