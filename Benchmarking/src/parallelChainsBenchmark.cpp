#include "pinocchioHelpers.hpp"

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
        parallel_chains.push_back({5, i, 1e-8, "Explicit"});
        parallel_chains.push_back({5, i + 1, 1e-4, "Implicit"});
    }
    for (int i : {2, 4, 8, 12, 16})
    {
        parallel_chains.push_back({10, i, 1e-6, "Explicit"});
        parallel_chains.push_back({10, i + 1, 1e-3, "Implicit"});
    }
    for (int i : {2, 6, 12, 20, 30})
    {
        parallel_chains.push_back({20, i, 1e-2, "Explicit"});
        parallel_chains.push_back({20, i + 1, 1e1, "Implicit"});
    }
    for (int i : {2, 8, 16, 28, 40})
    {
        parallel_chains.push_back({40, i, 5e0, "Explicit"});
        parallel_chains.push_back({40, i + 1, 1e2, "Implicit"});
    }

    return parallel_chains;
}

class PinocchioParallelChainsNumericalValidation : public ::testing::TestWithParam<ParallelChainSpecification>
{
protected:
    const int num_samples = 5;
};

INSTANTIATE_TEST_SUITE_P(PinocchioParallelChainsNumericalValidation,
                         PinocchioParallelChainsNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkUrdfFiles()));

TEST_P(PinocchioParallelChainsNumericalValidation, forward_dynamics)
{
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    // urdf file name
    std::string urdf_filename = urdf_directory + "parallel_chains/" +
                                GetParam().constraint_type + "/depth" +
                                std::to_string(GetParam().depth) + "/loop_size" +
                                std::to_string(GetParam().loop_size) + ".urdf";

    // Build models
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);

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

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);
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
        const Eigen::MatrixXd K_pinocchio = K_cluster * joint_map.transpose();
        const Eigen::VectorXd k_pinocchio = -k_cluster;

        // Compute the forward dynamics
        Eigen::VectorXd spanning_joint_tau = Eigen::VectorXd::Random(nv_span);
        Eigen::VectorXd tau = G_cluster.transpose() * spanning_joint_tau;

        const double mu0 = 1e-14;
        Eigen::VectorXd pin_tau(nv_span);
        pin_tau = joint_map * spanning_joint_tau;

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
        GTEST_ASSERT_LT(grbda_error.norm(), GetParam().tol)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const Eigen::VectorXd pin_error = data.ddq - joint_map * qdd_lgm;
        GTEST_ASSERT_LT(pin_error.norm(), GetParam().tol)
            << "qdd_pinocchio   : " << data.ddq.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map * qdd_lgm).transpose();

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), GetParam().tol)
            << "K*qdd_lgm + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), GetParam().tol)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }

}

class PinocchioParallelChainsBenchmark : public ::testing::TestWithParam<ParallelChainSpecification>
{
public:
    std::ofstream outfile;
    grbda::Timer timer;
    const int num_samples = 10;

private:
    void SetUp() override
    {
        std::string csv_filename = "InstructionPinocchioFD_" + GetParam().constraint_type + ".csv";
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
    double t_pinocchio = 0.;

    // urdf file name
    std::string urdf_filename = urdf_directory + "parallel_chains/" +
                                GetParam().constraint_type + "/depth" +
                                std::to_string(GetParam().depth) + "/loop_size" +
                                std::to_string(GetParam().loop_size) + ".urdf";

    // Build models
    PinocchioModel model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADModel::Data ad_data(ad_model);

    grbda::ClusterTreeModel<ADScalar> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename, false);

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

    pinocchio::forwardDynamics(ad_model, ad_data, pin_q, pin_v, pin_tau, K_pinocchio, k_pinocchio, mu0);

    const DynamicADVector qdd_lgm = lgm_model.forwardDynamics(tau);

    // Create casadi function to compute forward dynamics
    ADScalar cs_qdd_cluster = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_cluster, cs_qdd_cluster);

    ADScalar cs_qdd_lgm = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_lgm, cs_qdd_lgm);

    ADScalar cs_qdd_pinocchio = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(ad_data.ddq, cs_qdd_pinocchio);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_qdd_cluster});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio});

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
        numerical_cluster_tree.buildModelFromURDF(urdf_filename, false);

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

        casadi::DMVector dm_lgm_res = csGrbdaLgm(casadi::DMVector{dm_q, dm_v, dm_tau});

        DynamicVector cABA_res(nv_span), pin_res(nv_span), lgm_res(nv_span);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_pin_res[0], pin_res);
        casadi::copy(dm_lgm_res[0], lgm_res);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        const DynamicVector grbda_error = lgm_res - cABA_res;
        GTEST_ASSERT_LT(grbda_error.norm(), GetParam().tol)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << cABA_res.transpose() << "\n"
            << "cABA_res: " << cABA_res.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DynamicVector qdd_error = pin_res - joint_map.cast<Scalar>() * lgm_res;
        GTEST_ASSERT_LT(qdd_error.norm(), GetParam().tol)
            << "qdd_pinocchio   : " << pin_res.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.cast<Scalar>() * lgm_res).transpose();
    }

    // csv format: depth, loop size , cluster_instr, pinocchio_instr, cluster_time, pinocchio_time
    outfile << GetParam().depth << "," << GetParam().loop_size << ","
            << csClusterABA.n_instructions() << "," << csPinocchioFD.n_instructions() << ","
            << t_cluster / num_samples << "," << t_pinocchio / num_samples << std::endl;
}
