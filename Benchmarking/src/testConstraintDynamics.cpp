#include "pinocchioHelpers.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"

TEST(TestConstraintDynamics, NumericalFourBar)
{
    // Aliases
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

    // URDF File
    std::string urdf_filename = main_urdf_directory + "four_bar.urdf";

    // Build Models
    PinModel<double> model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // TODO(@MatthewChignoli): Better names for these?
    PinData<double> data(model);    // For forward dynamics
    PinData<double> data_cd(model); // For constrained dynamics

    auto pin_constraints = parseURDFFileForLoopConstraints(urdf_filename, model);
    ConstraintModelVector<double> constraint_models = pin_constraints.models;
    ConstraintDataVector<double> constraint_datas = pin_constraints.datas;
    const double mu0 = 1e-14;
    pinocchio::ProximalSettings prox_settings(1e-12, mu0, 4);
    pinocchio::initConstraintDynamics(model, data_cd, constraint_models);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);

    // Extract info from models
    JointMap<double> joint_map = jointMap(lgm_model, model);
    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    const double tol = 5e-9;
    // TODO(@MatthewChignoli): The next lines should be in a loop over the number of samples
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
            {
                ASSERT_TRUE(constraint->isValidSpanningPosition(spanning_joint_state.position));
            }
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
        pin_v = joint_map.vel * spanning_joint_vel;

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
            GTEST_ASSERT_LT(KG.norm(), 1e-8) << "K*G: " << KG.transpose();
            GTEST_ASSERT_LT(Kg_k.norm(), 1e-8) << "K*g - k: " << Kg_k.transpose();

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

        Eigen::VectorXd pin_tau(nv_span);
        pin_tau = joint_map.vel * spanning_joint_tau;

        const Eigen::VectorXd ydd_cluster = cluster_tree.forwardDynamics(tau);
        Eigen::VectorXd qdd_cluster = Eigen::VectorXd::Zero(0);
        for (const auto &cluster : cluster_tree.clusters())
        {
            Eigen::VectorXd ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                        cluster->num_velocities_);
            Eigen::VectorXd qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
            qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
        }

        const Eigen::VectorXd qdd_lgm = lgm_model.forwardDynamics(tau);

        pinocchio::forwardDynamics(model, data, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, mu0);

        pinocchio::constraintDynamics(model, data_cd, pin_q, pin_v, pin_tau,
                                      constraint_models, constraint_datas, prox_settings);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const Eigen::VectorXd grbda_error = qdd_lgm - qdd_cluster;
        GTEST_ASSERT_LT(grbda_error.norm(), tol)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const Eigen::VectorXd pin_error = data.ddq - joint_map.vel * qdd_lgm;
        GTEST_ASSERT_LT(pin_error.norm(), tol)
            << "qdd_pinocchio   : " << data.ddq.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.vel * qdd_lgm).transpose();

        // Check the pinocchio constrained dynamics solution against the pinocchio forward dynamics
        const Eigen::VectorXd pin_error_2 = data.ddq - data_cd.ddq;
        GTEST_ASSERT_LT(pin_error_2.norm(), tol)
            << "qdd_pinocchio_fd: " << data.ddq.transpose() << "\n"
            << "qdd_pinocchio_cd: " << data_cd.ddq.transpose();
        std::cout << "qdd_pinocchio (forwardDynamics): " << data.ddq.transpose() << "\n";
        std::cout << "qdd_pinocchio (constraintDynamics): " << data_cd.ddq.transpose() << "\n";

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), tol)
            << "K*qdd_lgm + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), tol)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }
}

TEST(TestConstraintDynamics, SymbolicFourBar)
{
    // Aliases
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

    using ProximalSettings = pinocchio::ProximalSettingsTpl<ADScalar>;

    // URDF File
    std::string urdf_filename = main_urdf_directory + "four_bar.urdf";

    // Build models
    PinocchioModel model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    PinocchioADModel ad_model = model.cast<ADScalar>();
    PinocchioADModel::Data ad_data(ad_model);
    PinocchioADModel::Data ad_data_cd(ad_model);

    auto pin_constraints = parseURDFFileForLoopConstraints(urdf_filename, ad_model);
    ConstraintModelVector<ADScalar> constraint_models = pin_constraints.models;
    ConstraintDataVector<ADScalar> constraint_datas = pin_constraints.datas;
    const ADScalar mu0 = 1e-14;
    ProximalSettings prox_settings(1e-12, mu0, 4);
    pinocchio::initConstraintDynamics(ad_model, ad_data_cd, constraint_models);

    grbda::ClusterTreeModel<ADScalar> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<ADScalar>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);

    // Symbolic state
    const int nq = cluster_tree.getNumPositions();
    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();
    JointMap<ADScalar> joint_map = jointMap(lgm_model, model);

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
    pin_v = joint_map.vel * spanning_joint_vel;

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

    DynamicADVector pin_tau(nv_span);
    pin_tau = joint_map.vel * spanning_joint_tau;

    const DynamicADVector ydd_cluster = cluster_tree.forwardDynamics(tau);
    DynamicADVector qdd_cluster = DynamicADVector::Zero(0);
    for (const auto &cluster : cluster_tree.clusters())
    {
        DynamicADVector ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                    cluster->num_velocities_);
        DynamicADVector qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
        qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
    }

    const DynamicADVector qdd_lgm = lgm_model.forwardDynamics(tau);

    pinocchio::forwardDynamics(ad_model, ad_data, pin_q, pin_v, pin_tau,
                               K_pinocchio, k_pinocchio, mu0);

    pinocchio::constraintDynamics(ad_model, ad_data_cd, pin_q, pin_v, pin_tau,
                                  constraint_models, constraint_datas, prox_settings);

    // Create casadi function to compute forward dynamics
    ADScalar cs_qdd_cluster = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_cluster, cs_qdd_cluster);

    ADScalar cs_qdd_lgm = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_lgm, cs_qdd_lgm);

    ADScalar cs_qdd_pinocchio_fd = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(ad_data.ddq, cs_qdd_pinocchio_fd);

    ADScalar cs_qdd_pinocchio_cd = ADScalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(ad_data_cd.ddq, cs_qdd_pinocchio_cd);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_qdd_cluster});

    casadi::Function csGrbdaLgm("grbdaLgm",
                                casadi::SXVector{cs_q, cs_v, cs_tau},
                                casadi::SXVector{cs_qdd_lgm});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio_fd});

    casadi::Function csPinocchioCD("pinocchioCD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio_cd});

    // Check the solutions against each other
    using ScalarModel = grbda::ClusterTreeModel<Scalar>;
    using ScalarModelState = grbda::ModelState<Scalar>;
    using ScalarJointState = grbda::JointState<Scalar>;
    using ScalarStatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
    
    const int num_samples = 50;
    const double tol = 5e-9;

    grbda::Timer timer;
    double t_cluster = 0.;
    double t_pin_fd = 0.;
    double t_pin_cd = 0.;
    
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
        casadi::DMVector dm_lgm_res = csGrbdaLgm(casadi::DMVector{dm_q, dm_v, dm_tau});
        timer.start();
        casadi::DMVector dm_pin_fd_res = csPinocchioFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_fd += timer.getMs();
        timer.start();
        casadi::DMVector dm_pin_cd_res = csPinocchioCD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_cd += timer.getMs();

        DynamicVector cABA_res(nv_span), lgm_res(nv_span), pin_fd_res(nv_span), pin_cd_res(nv_span);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_lgm_res[0], lgm_res);
        casadi::copy(dm_pin_fd_res[0], pin_fd_res);
        casadi::copy(dm_pin_cd_res[0], pin_cd_res);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        const DynamicVector grbda_error = lgm_res - cABA_res;
        GTEST_ASSERT_LT(grbda_error.norm(), tol)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << cABA_res.transpose() << "\n"
            << "cABA_res: " << cABA_res.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DynamicVector qdd_error = pin_fd_res - joint_map.vel.cast<Scalar>() * lgm_res;
        GTEST_ASSERT_LT(qdd_error.norm(), tol)
            << "qdd_pinocchio   : " << pin_fd_res.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.vel.cast<Scalar>() * lgm_res).transpose();

        // Check the pinocchio constrained dynamics solution against the pinocchio forward dynamics
        const DynamicVector pin_error = pin_fd_res - pin_cd_res;
        GTEST_ASSERT_LT(pin_error.norm(), tol)
            << "qdd_pinocchio_fd: " << pin_fd_res.transpose() << "\n"
            << "qdd_pinocchio_cd: " << pin_cd_res.transpose();
    }

    // Print the numbers of instructions
    std::cout << "Num Instr - cluster ABA: " << csClusterABA.n_instructions() << "\n";
    std::cout << "Num Instr - pinocchio FD: " << csPinocchioFD.n_instructions() << "\n";
    std::cout << "Num Instr - pinocchio CD: " << csPinocchioCD.n_instructions() << "\n";
    std::cout << "Num Instr - grbda LGM: " << csGrbdaLgm.n_instructions() << "\n";

    // Print the timings
    std::cout << "Time - cluster ABA: " << t_cluster / num_samples << " ms\n";
    std::cout << "Time - pinocchio FD: " << t_pin_fd / num_samples << " ms\n";
    std::cout << "Time - pinocchio CD: " << t_pin_cd / num_samples << " ms\n";
}
