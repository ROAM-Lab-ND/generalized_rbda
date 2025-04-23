#include "pinocchioHelpers.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"

// TODO(@MatthewChignoli): Merge as much of this cose as possible with rotorBenchmark.cpp

struct ParallelChainSpecification
{
    int depth;
    int loop_size;
    double tol;
    std::string constraint_type;
};

std::vector<ParallelChainSpecification> GetBenchmarkUrdfFiles()
{
    for (std::string ctype : {"Explicit", "Implicit"})
    {
        std::ofstream outfile;
        outfile.open(path_to_data + "InstructionPinocchioFD_" + ctype + ".csv", std::ios::trunc);
        if (!outfile.is_open())
        {
            std::cerr << "Failed to open file." << std::endl;
        }
        if (outfile.is_open())
        {
            outfile.close();
        }
    }

    std::vector<ParallelChainSpecification> parallel_chains;
    for (int i : {2, 4, 6, 8, 10})
    {
        //parallel_chains.push_back({5, i, 1e-8, "Explicit"});
        parallel_chains.push_back({5, i + 1, 1e-4, "Implicit"});
    }
    for (int i : {2, 4, 8, 12, 16})
    {
        //parallel_chains.push_back({10, i, 1e-6, "Explicit"});
        parallel_chains.push_back({10, i + 1, 1e-3, "Implicit"});
    }
    for (int i : {2, 6, 12, 20, 30})
    {
        //parallel_chains.push_back({20, i, 1e-2, "Explicit"});
        parallel_chains.push_back({20, i + 1, 1e1, "Implicit"});
    }
    for (int i : {2, 8, 16, 28, 40})
    {
        //parallel_chains.push_back({40, i, 5e0, "Explicit"});
        parallel_chains.push_back({40, i + 1, 1e2, "Implicit"});
    }

    return parallel_chains;
}

/*class PinocchioParallelChainsNumericalValidation : public ::testing::TestWithParam<ParallelChainSpecification>
{
protected:
    const int num_samples = 5;
};

INSTANTIATE_TEST_SUITE_P(PinocchioParallelChainsNumericalValidation,
                         PinocchioParallelChainsNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioParallelChainsNumericalValidation, forward_dynamics)
*/

class PinocchioParallelChainsNumericalValidation : public ::testing::Test
{
protected:
    const int num_samples = 5;
    const double tol = 1e-4;
    const int KP = 10;
    const int KD = 10;
};

TEST_F(PinocchioParallelChainsNumericalValidation, forward_dynamics)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    // urdf file name
    /*
    std::string urdf_filename = urdf_directory + "parallel_chains/" +
                                GetParam().constraint_type + "/depth" +
                                std::to_string(GetParam().depth) + "/loop_size" +
                                std::to_string(GetParam().loop_size) + ".urdf";
    */
    std::string urdf_filename = urdf_directory + "parallel_chains/Implicit/depth5/loop_size5.urdf";

    // Build models
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);
    pinocchio::Data data_cd(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename);

    // Check that the cluster tree is properly formed
    /*int expected_bodies = GetParam().constraint_type == "Explicit" ? 2 * GetParam().depth
                                                                   : 2 * GetParam().depth + 1;
    GTEST_ASSERT_EQ(cluster_tree.getNumBodies(), expected_bodies);
    int largest_loop_size = -1;
    for (const auto &cluster : cluster_tree.clusters())
    {
        int loop_size = cluster->bodies_.size();
        if (loop_size > largest_loop_size)
            largest_loop_size = loop_size;
    }
    GTEST_ASSERT_EQ(largest_loop_size, GetParam().loop_size);*/

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
            GTEST_ASSERT_LT(KG.norm(), 1e-8)
                << "K*G: " << KG.transpose();
            GTEST_ASSERT_LT(Kg_k.norm(), 1e-8)
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

        // Constraint dynamics
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModel) constraint_models;
        PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintData) constraint_data;
        const std::string RA = "connecting_rod_joint";
        const pinocchio::Model::JointIndex RA_id = model.getJointId(RA);
        const std::string LA = "joint_2_2";
        const pinocchio::Model::JointIndex LA_id = model.getJointId(LA);

        // Add loop closure constraint
        Eigen::Vector3d p_loop_from_rod, p_loop_from_j22;
        p_loop_from_rod << 1., 0., 0.;
        p_loop_from_j22 << 0., 1., 0.;
        pinocchio::SE3 SE3_from_rod_joint = pinocchio::SE3(Eigen::Matrix3d::Identity(), p_loop_from_rod);
        pinocchio::SE3 SE3_from_joint_2_2 = pinocchio::SE3(Eigen::Matrix3d::Identity(), p_loop_from_j22);
        pinocchio::RigidConstraintModel ci_closure(
            pinocchio::ContactType::CONTACT_6D, model, LA_id, SE3_from_rod_joint, RA_id, SE3_from_joint_2_2, pinocchio::ReferenceFrame::LOCAL);

        ci_closure.corrector.Kp.array() = KP; // value used in the unit test
        ci_closure.corrector.Kd.array() = KD; // value used in the unit test

        constraint_models.push_back(ci_closure);
        constraint_data.push_back(pinocchio::RigidConstraintData(ci_closure));
        //const double mu0 = 0.; // mu has been declared above to be 1e-14
        pinocchio::ProximalSettings prox_settings(1e-12, mu0, 100);

        pinocchio::initConstraintDynamics(model, data_cd, constraint_models);
        const Eigen::VectorXd qdd_cd =
            pinocchio::constraintDynamics(model, data_cd, pin_q, pin_v, pin_tau, constraint_models, constraint_data, prox_settings);

        pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, mu0);

        const Eigen::VectorXd qdd_lgm = lgm_model.forwardDynamics(tau);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const Eigen::VectorXd grbda_error = qdd_lgm - qdd_cluster;
        GTEST_ASSERT_LT(grbda_error.norm(), tol)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const Eigen::VectorXd pin_error = data.ddq - joint_map.vel* qdd_lgm;
        GTEST_ASSERT_LT(pin_error.norm(), tol)
            << "qdd_pinocchio   : " << data.ddq.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.vel* qdd_lgm).transpose();

        // Check grbda lagrange multiplier solution against pinocchio constrainedDynamics
        const Eigen::VectorXd pin_error_2 = data.ddq - qdd_cd;
        GTEST_ASSERT_LT(pin_error_2.norm(), tol)
            << "qdd_pinocchio_fd: " << data.ddq.transpose() << "\n"
            << "qdd_pinocchio_cd: " << qdd_cd.transpose();
        std::cout << "qdd_pinocchio (forwardDynamics): " << data.ddq.transpose() << "\n";
        std::cout << "qdd_pinocchio (constraintDynamics): " << qdd_cd.transpose() << "\n";

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), tol)
            << "K*qdd_lgm + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), tol)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }

}

class PinocchioParallelChainsBenchmark : public ::testing::TestWithParam<ParallelChainSpecification>
{
public:
    std::ofstream outfile;
    grbda::Timer timer;
    const int num_samples = 10;
    const int KP = 10;
    const int KD = 10;
    const double tol = 1e-4;
    const int prox_max_iter = 1000;

private:
    void SetUp() override
    {
        //std::string csv_filename = "InstructionPinocchioFD_" + GetParam().constraint_type + ".csv";
        std::string csv_filename = "InstructionPinocchioFD_Implicit_ConstraintDynamics_" + std::to_string(prox_max_iter) + ".csv";
        outfile.open(path_to_data + csv_filename, std::ios::app);
        if (!outfile.is_open())
        {
            std::cerr << "Failed to open file." << std::endl;
        }
    }

    void TearDown() override
    {
        if (outfile.is_open())
        {
            outfile.close();
        }
    }
};

INSTANTIATE_TEST_SUITE_P(PinocchioParallelChainsBenchmark, PinocchioParallelChainsBenchmark,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioParallelChainsBenchmark, compareInstructionCount)
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
    double t_pinocchio_fd = 0.;
    double t_pinocchio_cd = 0.;

    // urdf file name
    std::string urdf_filename = urdf_directory + "parallel_chains/" +
                                GetParam().constraint_type + "/depth" +
                                std::to_string(GetParam().depth) + "/loop_size" +
                                std::to_string(GetParam().loop_size) + ".urdf";
    //std::string urdf_filename = urdf_directory + "parallel_chains/Implicit/depth5/loop_size5.urdf";


    // Build models
    PinocchioModel model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADModel::Data ad_data(ad_model);
    PinocchioADModel::Data ad_data_cd(ad_model);

    grbda::ClusterTreeModel<ADScalar> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename);

    // Check that the cluster tree is properly formed
    int expected_bodies = GetParam().constraint_type == "Explicit" ? 2 * GetParam().depth
                                                                   : 2 * GetParam().depth + 1;
    GTEST_ASSERT_EQ(cluster_tree.getNumBodies(), expected_bodies);
    int largest_loop_size = -1;
    for (const auto &cluster : cluster_tree.clusters())
    {
        int loop_size = cluster->bodies_.size();
        if (loop_size > largest_loop_size)
            largest_loop_size = loop_size;
    }
    GTEST_ASSERT_EQ(largest_loop_size, GetParam().loop_size);

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

    // TODO(@MatthewChignoli): Do we need to include the conversion to qdd for cABA?
    const DynamicADVector ydd_cluster = cluster_tree.forwardDynamics(tau);
    DynamicADVector qdd_cluster = DynamicADVector::Zero(0);
    for (const auto &cluster : cluster_tree.clusters())
    {
        DynamicADVector ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                    cluster->num_velocities_);
        DynamicADVector qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();  
        qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
    }

    // Constraint dynamics
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintModelTpl<ADScalar>) constraint_models;
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidConstraintDataTpl<ADScalar>) constraint_data;
    const std::string RA = "connecting_rod_joint";
    const pinocchio::Model::JointIndex RA_id = model.getJointId(RA);
    const std::string LA = "joint_2_2";
    const pinocchio::Model::JointIndex LA_id = model.getJointId(LA);

    // Add loop closure constraint
    using Vector3AD = Eigen::Vector<ADScalar, 3>;
    using Matrix3AD = Eigen::Matrix<ADScalar, 3, 3>;
    Vector3AD p_loop_from_rod, p_loop_from_j22;
    p_loop_from_rod << 1., 0., 0.;
    p_loop_from_j22 << 0., 1., 0.;
    Matrix3AD I = Matrix3AD::Identity();
    pinocchio::SE3Tpl<ADScalar> SE3_from_rod_joint = pinocchio::SE3Tpl<ADScalar>(I, p_loop_from_rod);
    pinocchio::SE3Tpl<ADScalar> SE3_from_joint_2_2 = pinocchio::SE3Tpl<ADScalar>(I, p_loop_from_j22);
    
    pinocchio::RigidConstraintModelTpl<ADScalar> ci_closure(
        pinocchio::ContactType::CONTACT_6D, ad_model, LA_id, SE3_from_rod_joint, RA_id, SE3_from_joint_2_2, pinocchio::ReferenceFrame::LOCAL);

    ci_closure.corrector.Kp.array() = KP; // value used in the unit test
    ci_closure.corrector.Kd.array() = KD; // value used in the unit test

    constraint_models.push_back(ci_closure);
    constraint_data.push_back(pinocchio::RigidConstraintDataTpl<ADScalar>(ci_closure));
    const ADScalar prox_accuracy = 1e-12;
    const ADScalar prox_mu0 = 0;
    pinocchio::ProximalSettingsTpl<ADScalar> prox_settings(prox_accuracy, prox_mu0, prox_max_iter);

    pinocchio::initConstraintDynamics(ad_model, ad_data_cd, constraint_models);
    const DynamicADVector qdd_cd =
        pinocchio::constraintDynamics(ad_model, ad_data_cd, pin_q, pin_v, pin_tau, constraint_models, constraint_data, prox_settings);

    pinocchio::forwardDynamics(ad_model, ad_data, pin_q, pin_v, pin_tau, K_pinocchio, k_pinocchio, mu0);

    const DynamicADVector qdd_lgm = lgm_model.forwardDynamics(tau);

    // Create casadi function to compute forward dynamics
    ADScalar cs_qdd_cluster = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_cluster, cs_qdd_cluster);

    ADScalar cs_qdd_lgm = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_lgm, cs_qdd_lgm);

    ADScalar cs_qdd_pinocchio = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(ad_data.ddq, cs_qdd_pinocchio);

    ADScalar cs_qdd_pinocchio_cd = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_cd, cs_qdd_pinocchio_cd);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_qdd_cluster});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio});

    casadi::Function csPinocchioCD("pinocchioCD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio_cd});

    casadi::Function csGrbdaLgm("grbdaLgm",
                               casadi::SXVector{cs_q, cs_v, cs_tau},
                               casadi::SXVector{cs_qdd_lgm});

    // Check the solutions against each other
    using ScalarModel = grbda::ClusterTreeModel<Scalar>;
    using ScalarModelState = grbda::ModelState<Scalar>;
    using ScalarJointState = grbda::JointState<Scalar>;
    using ScalarStatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
    for (int i = 0; i < num_samples; ++i)
    {
        ScalarModel numerical_cluster_tree;
        numerical_cluster_tree.buildModelFromURDF(urdf_filename);

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
        t_pinocchio_fd += timer.getMs();

        timer.start();
        casadi::DMVector dm_pin_cd_res = csPinocchioCD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pinocchio_cd += timer.getMs();

        casadi::DMVector dm_lgm_res = csGrbdaLgm(casadi::DMVector{dm_q, dm_v, dm_tau});

        DynamicVector cABA_res(nv_span), pin_res(nv_span), lgm_res(nv_span), pin_res_cd(nv_span);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_pin_res[0], pin_res);
        casadi::copy(dm_lgm_res[0], lgm_res);
        casadi::copy(dm_pin_cd_res[0], pin_res_cd);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        /*const DynamicVector grbda_error = lgm_res - cABA_res;
        GTEST_ASSERT_LT(grbda_error.norm(), tol)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << cABA_res.transpose() << "\n"
            << "cABA_res: " << cABA_res.transpose();*/

        // Check grbda lagrange multiplier solution against pinocchio
        /*const DynamicVector qdd_error = pin_res - joint_map.vel.cast<Scalar>() * lgm_res;
        GTEST_ASSERT_LT(qdd_error.norm(), tol)
            << "qdd_pinocchio   : " << pin_res.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.vel.cast<Scalar>() * lgm_res).transpose();*/
    }

    // csv format: depth, loop size , proximal and sparse max iter, proximal and sparse total iter, cluster_instr, pinocchio_instr, cluster_time, pinocchio_time
    outfile << GetParam().depth << "," << GetParam().loop_size << "," << prox_max_iter << "," << prox_settings.iter << ","
            << csClusterABA.n_instructions() << "," << csPinocchioFD.n_instructions() << ","
            << csPinocchioCD.n_instructions() << ","
            << t_cluster / num_samples << "," << t_pinocchio_fd / num_samples << ","
            << t_pinocchio_cd / num_samples << std::endl;

}
