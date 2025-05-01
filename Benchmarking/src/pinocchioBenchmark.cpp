#include "pinocchioHelpers.hpp"

template <typename Scalar>
class PinocchioBase : public ::testing::TestWithParam<std::shared_ptr<RobotSpecification>>
{
protected:
    using ModelState = grbda::ModelState<Scalar>;
    using JointCoordinate = grbda::JointCoordinate<Scalar>;
    using JointState = grbda::JointState<Scalar>;
    using LoopConstraint = grbda::LoopConstraint::Base<Scalar>;
    using LoopConstraintPtr = std::shared_ptr<grbda::LoopConstraint::Base<Scalar>>;
    using StatePair = std::pair<DVec<Scalar>, DVec<Scalar>>;

    PinocchioBase(int num_samples) : num_samples_(num_samples),
                                     qdd_tol_(GetParam()->qdd_tol_),
                                     qdd_cd_tol_(GetParam()->qdd_cd_tol_),
                                     cnstr_tol_(GetParam()->cnstr_tol_),
                                     cluster_tree_(GetParam()->urdf_filename),
                                     approx_cluster_tree_(GetParam()->approx_urdf_filename),
                                     lgm_model_(cluster_tree_,
                                                grbda::FwdDynMethod::LagrangeMultiplierEigen),
                                     approx_lgm_model_(approx_cluster_tree_,
                                                       grbda::FwdDynMethod::LagrangeMultiplierEigen)
    {
        PinModel<double> tmp_model;
        pinocchio::urdf::buildModel(GetParam()->urdf_filename, tmp_model);
        joint_map_ = jointMap(lgm_model_, tmp_model);

        PinModel<double> approx_tmp_model;
        pinocchio::urdf::buildModel(GetParam()->approx_urdf_filename, approx_tmp_model);
        approx_joint_map_ = jointMap(approx_lgm_model_, approx_tmp_model);

        pin_model_ = tmp_model.cast<Scalar>();
        approx_pin_model_ = approx_tmp_model.cast<Scalar>();

        pin_data_fd_ = PinData<Scalar>(pin_model_);
        pin_data_cd1_ = PinData<Scalar>(pin_model_);
        pin_data_cd2_ = PinData<Scalar>(pin_model_);
        pin_data_cd5_ = PinData<Scalar>(pin_model_);
        approx_pin_data_ = PinData<Scalar>(approx_pin_model_);

        auto pin_constraints = parseURDFFileForLoopConstraints(GetParam()->urdf_filename,
                                                               pin_model_);
        constraint_models_ = pin_constraints.models;
        constraint_datas_ = pin_constraints.datas;
        pinocchio::initConstraintDynamics(pin_model_, pin_data_cd1_, constraint_models_);
        pinocchio::initConstraintDynamics(pin_model_, pin_data_cd2_, constraint_models_);
        pinocchio::initConstraintDynamics(pin_model_, pin_data_cd5_, constraint_models_);

        const Scalar prox_accuracy = 1e-12;
        const Scalar prox_mu = 1e-10; // What value to use here?
        prox_settings1_ = PinProxSettings<Scalar>(prox_accuracy, prox_mu, 1);
        prox_settings2_ = PinProxSettings<Scalar>(prox_accuracy, prox_mu, 2);
        prox_settings5_ = PinProxSettings<Scalar>(prox_accuracy, prox_mu, 5);
    }

    const int num_samples_;
    const double qdd_tol_;
    const double qdd_cd_tol_;
    const double cnstr_tol_;

    const Scalar fd_mu_ = 1e-14;
    PinProxSettings<Scalar> prox_settings1_, prox_settings2_, prox_settings5_;

    PinModel<Scalar> pin_model_, approx_pin_model_;
    PinData<Scalar> pin_data_fd_, pin_data_cd1_, pin_data_cd2_, pin_data_cd5_, approx_pin_data_;

    ConstraintModelVector<Scalar> constraint_models_;
    ConstraintDataVector<Scalar> constraint_datas_;

    grbda::ClusterTreeModel<Scalar> cluster_tree_, approx_cluster_tree_;
    grbda::RigidBodyTreeModel<Scalar> lgm_model_, approx_lgm_model_;

    JointMap<Scalar> joint_map_, approx_joint_map_;
};

class PinocchioNumericalValidation : public PinocchioBase<double>
{
protected:
    using Scalar = double;
    PinocchioNumericalValidation() : PinocchioBase<double>(5) {}

    StatePair createRandomState()
    {
        ModelState model_state;
        ModelState spanning_model_state;
        for (const auto &cluster : cluster_tree_.clusters())
        {
            JointState joint_state = cluster->joint_->randomJointState();
            JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            std::shared_ptr<LoopConstraint> constraint = cluster->joint_->cloneLoopConstraint();
            if (!constraint->isExplicit())
            {
                if (!constraint->isValidSpanningPosition(spanning_joint_state.position))
                {
                    throw std::runtime_error("Invalid spanning position");
                }
            }
            constraint->updateJacobians(spanning_joint_state.position);

            if (!constraint->isValidSpanningVelocity(spanning_joint_state.velocity))
            {
                throw std::runtime_error("Invalid spanning velocity");
            }

            model_state.push_back(joint_state);
            spanning_model_state.push_back(spanning_joint_state);
        }

        StatePair spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
        cluster_tree_.setState(model_state);
        lgm_model_.setState(spanning_q_and_v.first, spanning_q_and_v.second);
        return spanning_q_and_v;
    }
};

INSTANTIATE_TEST_SUITE_P(PinocchioNumericalValidation, PinocchioNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkRobotSpecifications()));

TEST_P(PinocchioNumericalValidation, forward_dynamics)
{
    for (int i = 0; i < num_samples_; i++)
    {
        StatePair spanning_q_and_v = createRandomState();
        DVec<Scalar> spanning_joint_pos = spanning_q_and_v.first;
        DVec<Scalar> spanning_joint_vel = spanning_q_and_v.second;

        const int nv_span = lgm_model_.getNumDegreesOfFreedom();
        DVec<Scalar> pin_q(nv_span), pin_v(nv_span);
        pin_q = joint_map_.pos * spanning_joint_pos;
        pin_v = joint_map_.vel * spanning_joint_vel;

        // Extract loop constraints from the cluster tree
        cluster_tree_.forwardKinematics();
        DMat<Scalar> K_cluster = DMat<Scalar>::Zero(0, 0);
        DVec<Scalar> k_cluster = DVec<Scalar>::Zero(0);
        DMat<Scalar> G_cluster = DMat<Scalar>::Zero(0, 0);
        DVec<Scalar> g_cluster = DVec<Scalar>::Zero(0);
        for (const auto &cluster : cluster_tree_.clusters())
        {
            // Check that the loop constraints are properly formed
            std::shared_ptr<LoopConstraint> constraint = cluster->joint_->cloneLoopConstraint();
            const DVec<Scalar> KG = constraint->K() * constraint->G();
            const DVec<Scalar> Kg_k = constraint->K() * constraint->g() - constraint->k();
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
        const DMat<Scalar> K_pinocchio = K_cluster * joint_map_.vel.transpose();
        const DVec<Scalar> k_pinocchio = -k_cluster;

        // Compute the forward dynamics
        DVec<Scalar> spanning_joint_tau = DVec<Scalar>::Random(nv_span);
        DVec<Scalar> tau = G_cluster.transpose() * spanning_joint_tau;

        DVec<Scalar> pin_tau(nv_span);
        pin_tau = joint_map_.vel * spanning_joint_tau;

        const DVec<Scalar> ydd_cluster = cluster_tree_.forwardDynamics(tau);
        DVec<Scalar> qdd_cluster = DVec<Scalar>::Zero(0);
        for (const auto &cluster : cluster_tree_.clusters())
        {
            DVec<Scalar> ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                     cluster->num_velocities_);
            DVec<Scalar> qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
            qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
        }

        const DVec<Scalar> qdd_lgm = lgm_model_.forwardDynamics(tau);

        pinocchio::forwardDynamics(pin_model_, pin_data_fd_, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, fd_mu_);

        pinocchio::constraintDynamics(pin_model_, pin_data_cd1_, pin_q, pin_v, pin_tau,
                                      constraint_models_, constraint_datas_, prox_settings1_);
        pinocchio::constraintDynamics(pin_model_, pin_data_cd2_, pin_q, pin_v, pin_tau,
                                      constraint_models_, constraint_datas_, prox_settings2_);
        pinocchio::constraintDynamics(pin_model_, pin_data_cd5_, pin_q, pin_v, pin_tau,
                                      constraint_models_, constraint_datas_, prox_settings5_);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const DVec<Scalar> grbda_error = qdd_lgm - qdd_cluster;
        GTEST_ASSERT_LT(grbda_error.norm(), qdd_tol_)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DVec<Scalar> pin_error = pin_data_fd_.ddq - joint_map_.vel * qdd_lgm;
        GTEST_ASSERT_LT(pin_error.norm(), qdd_tol_)
            << "qdd_pinocchio   : " << pin_data_fd_.ddq.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map_.vel * qdd_lgm).transpose();

        // Check the pinocchio constrained dynamics solution against the pinocchio forward dynamics
        const DVec<Scalar> pin_error_cd1 = pin_data_fd_.ddq - pin_data_cd1_.ddq;
        const DVec<Scalar> pin_error_cd2 = pin_data_fd_.ddq - pin_data_cd2_.ddq;
        const DVec<Scalar> pin_error_cd5 = pin_data_fd_.ddq - pin_data_cd5_.ddq;
        if (!constraint_models_.empty())
        {
            GTEST_ASSERT_LT(pin_error_cd1.norm(), qdd_cd_tol_)
                << "qdd_pinocchio_fd: " << pin_data_fd_.ddq.transpose() << "\n"
                << "qdd_pinocchio_cd: " << pin_data_cd1_.ddq.transpose();

            GTEST_ASSERT_LT(pin_error_cd2.norm(), qdd_cd_tol_)
                << "qdd_pinocchio_fd: " << pin_data_fd_.ddq.transpose() << "\n"
                << "qdd_pinocchio_cd: " << pin_data_cd2_.ddq.transpose();

            GTEST_ASSERT_LT(pin_error_cd5.norm(), qdd_cd_tol_)
                << "qdd_pinocchio_fd: " << pin_data_fd_.ddq.transpose() << "\n"
                << "qdd_pinocchio_cd: " << pin_data_cd5_.ddq.transpose();
        }

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * pin_data_fd_.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), cnstr_tol_)
            << "K*qdd_lgm + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), cnstr_tol_)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }
}

class PinocchioBenchmark : public PinocchioBase<casadi::SX>
{
protected:
    using Scalar = casadi::SX;
    PinocchioBenchmark() : PinocchioBase<casadi::SX>(150) {}

    grbda::Timer timer_;
    double t_cluster_ = 0.;
    double t_lg_ = 0.;
    double t_pin_fd_ = 0.;
    double t_pin_cd1_ = 0.;
    double t_pin_cd2_ = 0.;
    double t_pin_cd5_ = 0.;
    double t_aba_ = 0.;
    double t_pin_aba_ = 0.;

    double eL2_cluster_ = 0.;
    double eL2_lg_ = 0.;
    double eL2_pin_fd_ = 0.;
    double eL2_pin_cd1_ = 0.;
    double eL2_pin_cd2_ = 0.;
    double eL2_pin_cd5_ = 0.;
    double eL2_aba_ = 0.;
    double eL2_pin_aba_ = 0.;

    double eLinf_cluster_ = 0.;
    double eLinf_lg_ = 0.;
    double eLinf_pin_fd_ = 0.;
    double eLinf_pin_cd1_ = 0.;
    double eLinf_pin_cd2_ = 0.;
    double eLinf_pin_cd5_ = 0.;
    double eLinf_aba_ = 0.;
    double eLinf_pin_aba_ = 0.;
};

INSTANTIATE_TEST_SUITE_P(PinocchioBenchmark, PinocchioBenchmark,
                         ::testing::ValuesIn(GetBenchmarkRobotSpecifications()));

TEST_P(PinocchioBenchmark, forward_dynamics)
{
    // Symbolic state
    const int nq = cluster_tree_.getNumPositions();
    const int nv = cluster_tree_.getNumDegreesOfFreedom();
    const int nv_span = lgm_model_.getNumDegreesOfFreedom();

    const int nq_approx = approx_cluster_tree_.getNumPositions();
    const int nv_approx = approx_cluster_tree_.getNumDegreesOfFreedom();
    const int nv_span_approx = approx_lgm_model_.getNumDegreesOfFreedom();

    const int nq_input = std::max(nq, nq_approx);
    const int nv_input = std::max(nv, nv_approx);

    // TODO(@MatthewChignoli): Add an assert that nv_approx = nv_span_approx

    Scalar cs_q = Scalar::sym("q", nq_input);
    Scalar cs_v = Scalar::sym("v", nv_input);

    DVec<Scalar> q(nq_input), v(nv_input);
    casadi::copy(cs_q, q);
    casadi::copy(cs_v, v);

    ModelState model_state;
    ModelState spanning_model_state;
    for (const auto &cluster : cluster_tree_.clusters())
    {
        DVec<Scalar> q_i = q.segment(cluster->position_index_, cluster->num_positions_);
        DVec<Scalar> v_i = v.segment(cluster->velocity_index_, cluster->num_velocities_);

        JointState joint_state;
        LoopConstraintPtr loop_constraint = cluster->joint_->cloneLoopConstraint();
        joint_state.position = JointCoordinate(q_i, !loop_constraint->isExplicit());
        joint_state.velocity = JointCoordinate(v_i, false);

        JointState spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

        model_state.push_back(joint_state);
        spanning_model_state.push_back(spanning_joint_state);
    }

    ModelState approx_model_state;
    ModelState approx_spanning_model_state;
    for (const auto &cluster : approx_cluster_tree_.clusters())
    {
        DVec<Scalar> q_i = q.segment(cluster->position_index_, cluster->num_positions_);
        DVec<Scalar> v_i = v.segment(cluster->velocity_index_, cluster->num_velocities_);

        JointState approx_joint_state;
        LoopConstraintPtr loop_constraint = cluster->joint_->cloneLoopConstraint();
        if (loop_constraint)
        {
            approx_joint_state.position = JointCoordinate(q_i, !loop_constraint->isExplicit());
        }
        else
        {
            approx_joint_state.position = JointCoordinate(q_i, false);
        }
        approx_joint_state.velocity = JointCoordinate(v_i, false);

        JointState approx_spanning_joint_state =
            cluster->joint_->toSpanningTreeState(approx_joint_state);

        approx_model_state.push_back(approx_joint_state);
        approx_spanning_model_state.push_back(approx_spanning_joint_state);
    }

    cluster_tree_.setState(model_state);
    approx_cluster_tree_.setState(approx_model_state);

    StatePair spanning_q_and_v = grbda::modelStateToVector(spanning_model_state);
    DVec<Scalar> spanning_joint_pos = spanning_q_and_v.first;
    DVec<Scalar> spanning_joint_vel = spanning_q_and_v.second;
    lgm_model_.setState(spanning_joint_pos, spanning_joint_vel);
    DVec<Scalar> pin_q(nv_span), pin_v(nv_span);
    pin_q = joint_map_.pos * spanning_joint_pos;
    pin_v = joint_map_.vel * spanning_joint_vel;

    StatePair approx_spanning_q_and_v = grbda::modelStateToVector(approx_spanning_model_state);
    DVec<Scalar> approx_pin_q(nv_span_approx), approx_pin_v(nv_span_approx);
    approx_pin_q = approx_joint_map_.pos * approx_spanning_q_and_v.first;
    approx_pin_v = approx_joint_map_.vel * approx_spanning_q_and_v.second;

    // Extract loop constraints from the cluster tree
    cluster_tree_.forwardKinematics();
    DMat<Scalar> K_cluster = DMat<Scalar>::Zero(0, 0);
    DVec<Scalar> k_cluster = DVec<Scalar>::Zero(0);
    DMat<Scalar> G_cluster = DMat<Scalar>::Zero(0, 0);
    DVec<Scalar> g_cluster = DVec<Scalar>::Zero(0);
    for (const auto &cluster : cluster_tree_.clusters())
    {
        K_cluster = grbda::appendEigenMatrix(K_cluster, cluster->joint_->K());
        k_cluster = grbda::appendEigenVector(k_cluster, cluster->joint_->k());
        G_cluster = grbda::appendEigenMatrix(G_cluster, cluster->joint_->G());
        g_cluster = grbda::appendEigenVector(g_cluster, cluster->joint_->g());
    }

    // Convert implicit loop constraint to Pinocchio joint order
    const DMat<Scalar> K_pinocchio = K_cluster * joint_map_.vel.transpose();
    const DVec<Scalar> k_pinocchio = -k_cluster;

    // Compute the forward dynamics
    Scalar cs_tau = Scalar::sym("tau", nv_span);
    DVec<Scalar> spanning_joint_tau(nv_span);
    casadi::copy(cs_tau, spanning_joint_tau);
    DVec<Scalar> tau = G_cluster.transpose() * spanning_joint_tau;

    DVec<Scalar> pin_tau(nv_span);
    pin_tau = joint_map_.vel * spanning_joint_tau;

    DVec<Scalar> approx_tau = spanning_joint_tau.head(nv_approx);
    DVec<Scalar> approx_pin_tau(nv_span_approx);
    approx_pin_tau = approx_joint_map_.vel * spanning_joint_tau.head(nv_span_approx);

    const DVec<Scalar> ydd_cluster = cluster_tree_.forwardDynamics(tau);
    DVec<Scalar> qdd_cluster = DVec<Scalar>::Zero(0);
    for (const auto &cluster : cluster_tree_.clusters())
    {
        DVec<Scalar> ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                    cluster->num_velocities_);
        DVec<Scalar> qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
        qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
    }

    const DVec<Scalar> qdd_lgm = lgm_model_.forwardDynamics(tau);

    pinocchio::forwardDynamics(pin_model_, pin_data_fd_, pin_q, pin_v, pin_tau,
                               K_pinocchio, k_pinocchio, fd_mu_);

    pinocchio::constraintDynamics(pin_model_, pin_data_cd1_, pin_q, pin_v, pin_tau,
                                  constraint_models_, constraint_datas_, prox_settings1_);
    pinocchio::constraintDynamics(pin_model_, pin_data_cd2_, pin_q, pin_v, pin_tau,
                                  constraint_models_, constraint_datas_, prox_settings2_);
    pinocchio::constraintDynamics(pin_model_, pin_data_cd5_, pin_q, pin_v, pin_tau,
                                  constraint_models_, constraint_datas_, prox_settings5_);

    const DVec<Scalar> approx_qdd_cluster = approx_cluster_tree_.forwardDynamics(approx_tau);
    pinocchio::aba(approx_pin_model_, approx_pin_data_, approx_pin_q, approx_pin_v, approx_pin_tau);

    // Create casadi function to compute forward dynamics
    Scalar cs_qdd_cluster = Scalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_cluster, cs_qdd_cluster);

    Scalar cs_qdd_lgm = Scalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(qdd_lgm, cs_qdd_lgm);

    Scalar cs_qdd_pinocchio_fd = Scalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(pin_data_fd_.ddq, cs_qdd_pinocchio_fd);

    Scalar cs_qdd_pinocchio_cd1 = Scalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(pin_data_cd1_.ddq, cs_qdd_pinocchio_cd1);

    Scalar cs_qdd_pinocchio_cd2 = Scalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(pin_data_cd2_.ddq, cs_qdd_pinocchio_cd2);

    Scalar cs_qdd_pinocchio_cd5 = Scalar(casadi::Sparsity::dense(nv_span, 1));
    casadi::copy(pin_data_cd5_.ddq, cs_qdd_pinocchio_cd5);

    Scalar cs_qdd_aba = Scalar(casadi::Sparsity::dense(nv_approx, 1));
    casadi::copy(approx_qdd_cluster, cs_qdd_aba);

    Scalar cs_qdd_pinocchio_aba = Scalar(casadi::Sparsity::dense(nv_span_approx, 1));
    casadi::copy(approx_pin_data_.ddq, cs_qdd_pinocchio_aba);

    casadi::Function csClusterABA("clusterABA",
                                  casadi::SXVector{cs_q, cs_v, cs_tau},
                                  casadi::SXVector{cs_qdd_cluster});

    casadi::Function csGrbdaLgm("grbdaLgm",
                                casadi::SXVector{cs_q, cs_v, cs_tau},
                                casadi::SXVector{cs_qdd_lgm});

    casadi::Function csPinocchioFD("pinocchioFD",
                                   casadi::SXVector{cs_q, cs_v, cs_tau},
                                   casadi::SXVector{cs_qdd_pinocchio_fd});

    casadi::Function csPinocchioCD1("pinocchioCD1",
                                    casadi::SXVector{cs_q, cs_v, cs_tau},
                                    casadi::SXVector{cs_qdd_pinocchio_cd1});
    casadi::Function csPinocchioCD2("pinocchioCD2",
                                    casadi::SXVector{cs_q, cs_v, cs_tau},
                                    casadi::SXVector{cs_qdd_pinocchio_cd2});
    casadi::Function csPinocchioCD5("pinocchioCD5",
                                    casadi::SXVector{cs_q, cs_v, cs_tau},
                                    casadi::SXVector{cs_qdd_pinocchio_cd5});

    casadi::Function csGrbdaABA("grbdaABA",
                                casadi::SXVector{cs_q, cs_v, cs_tau},
                                casadi::SXVector{cs_qdd_aba});

    casadi::Function csPinocchioABA("pinocchioABA",
                                    casadi::SXVector{cs_q, cs_v, cs_tau},
                                    casadi::SXVector{cs_qdd_pinocchio_aba});

    const int i_cluster = static_cast<int>(csClusterABA.n_instructions());
    const int i_lg = static_cast<int>(csGrbdaLgm.n_instructions());
    const int i_pin_fd = static_cast<int>(csPinocchioFD.n_instructions());
    const int i_pin_cd1 = static_cast<int>(csPinocchioCD1.n_instructions());
    const int i_pin_cd2 = static_cast<int>(csPinocchioCD2.n_instructions());
    const int i_pin_cd5 = static_cast<int>(csPinocchioCD5.n_instructions());
    const int i_aba = static_cast<int>(csGrbdaABA.n_instructions());
    const int i_pin_aba = static_cast<int>(csPinocchioABA.n_instructions());

    // Check the solutions against each other
    grbda::ClusterTreeModel<double> numerical_cluster_tree;
    numerical_cluster_tree.buildModelFromURDF(GetParam()->urdf_filename);
    for (int i = 0; i < num_samples_; ++i)
    {
        grbda::ModelState<double> scalar_model_state;
        for (const auto &cluster : numerical_cluster_tree.clusters())
        {
            auto joint_state = cluster->joint_->randomJointState();
            auto spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);
            scalar_model_state.push_back(joint_state);
        }
        numerical_cluster_tree.setState(scalar_model_state);
        numerical_cluster_tree.forwardKinematics();

        auto scalar_q_and_v = grbda::modelStateToVector(scalar_model_state);
        DVec<double> scalar_q = scalar_q_and_v.first;
        casadi::DMVector dm_q = grbda::random<casadi::DM>(nq_input);
        for (int j = 0; j < scalar_q.size(); ++j)
        {
            dm_q[j] = scalar_q(j);
        }
        casadi::DMVector dm_v = grbda::random<casadi::DM>(nv_input);
        casadi::DMVector dm_tau = grbda::random<casadi::DM>(nv_span);

        timer_.start();
        casadi::DMVector dm_cABA_res = csClusterABA(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_cluster_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_lgm_res = csGrbdaLgm(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_lg_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_pin_fd_res = csPinocchioFD(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_fd_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_pin_cd1_res = csPinocchioCD1(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_cd1_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_pin_cd2_res = csPinocchioCD2(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_cd2_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_pin_cd5_res = csPinocchioCD5(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_cd5_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_aba_res = csGrbdaABA(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_aba_ += timer_.getMs();
        timer_.start();
        casadi::DMVector dm_pin_aba_res = csPinocchioABA(casadi::DMVector{dm_q, dm_v, dm_tau});
        t_pin_aba_ += timer_.getMs();

        DVec<double> cABA_res(nv_span), lgm_res(nv_span), pin_fd_res(nv_span);
        DVec<double> pin_cd1_res(nv_span), pin_cd2_res(nv_span), pin_cd5_res(nv_span);
        DVec<double> aba_res(nv_approx), pin_aba_res(nv_span_approx);
        casadi::copy(dm_cABA_res[0], cABA_res);
        casadi::copy(dm_lgm_res[0], lgm_res);
        casadi::copy(dm_pin_fd_res[0], pin_fd_res);
        casadi::copy(dm_pin_cd1_res[0], pin_cd1_res);
        casadi::copy(dm_pin_cd2_res[0], pin_cd2_res);
        casadi::copy(dm_pin_cd5_res[0], pin_cd5_res);
        casadi::copy(dm_aba_res[0], aba_res);
        casadi::copy(dm_pin_aba_res[0], pin_aba_res);

        // Check the cluster ABA solution against the Lagrange multiplier solution
        const DVec<double> caba_error = pin_fd_res - joint_map_.vel.cast<double>() * cABA_res;
        EXPECT_LT(caba_error.norm(), qdd_tol_)
            << "qdd_lgm: " << lgm_res.transpose() << "\n"
            << "qdd_cABA: " << cABA_res.transpose() << "\n"
            << "cABA_res: " << cABA_res.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DVec<double> lgm_error = pin_fd_res - joint_map_.vel.cast<double>() * lgm_res;
        EXPECT_LT(lgm_error.norm(), qdd_tol_)
            << "qdd_pinocchio   : " << pin_fd_res.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map_.vel.cast<double>() * lgm_res).transpose();

        eL2_cluster_ += caba_error.norm();
        eL2_lg_ += lgm_error.norm();

        eLinf_cluster_ += caba_error.lpNorm<Eigen::Infinity>();
        eLinf_lg_ += lgm_error.lpNorm<Eigen::Infinity>();

        // Check the aba solutions
        if (nv_approx == nv_span_approx)
        {
            const DVec<double> aba_error = pin_aba_res - approx_joint_map_.vel.cast<double>() * aba_res;
            EXPECT_LT(aba_error.norm(), qdd_tol_)
                << "qdd_pinocchio_aba: " << pin_aba_res.transpose() << "\n"
                << "jmap * qdd_aba: " << (approx_joint_map_.vel.cast<double>() * aba_res).transpose();

            eL2_aba_ += aba_error.norm();
            eLinf_aba_ += aba_error.lpNorm<Eigen::Infinity>();
        }
        else
        {
            eL2_aba_ = num_samples_ * -999;
            eLinf_aba_ = num_samples_ * -999;
        }

        // Check the pinocchio constrained dynamics solution against the pinocchio forward dynamics
        const DVec<double> pin1_error = pin_fd_res - pin_cd1_res;
        const DVec<double> pin2_error = pin_fd_res - pin_cd2_res;
        const DVec<double> pin5_error = pin_fd_res - pin_cd5_res;
        if (!constraint_models_.empty())
        {
            EXPECT_LT(pin1_error.norm(), qdd_cd_tol_)
                << "qdd_pinocchio_fd: " << pin_fd_res.transpose() << "\n"
                << "qdd_pinocchio_cd: " << pin_cd1_res.transpose();

            EXPECT_LT(pin2_error.norm(), qdd_cd_tol_)
                << "qdd_pinocchio_fd: " << pin_fd_res.transpose() << "\n"
                << "qdd_pinocchio_cd: " << pin_cd2_res.transpose();

            EXPECT_LT(pin5_error.norm(), qdd_cd_tol_)
                << "qdd_pinocchio_fd: " << pin_fd_res.transpose() << "\n"
                << "qdd_pinocchio_cd: " << pin_cd5_res.transpose();

            eL2_pin_cd1_ += pin1_error.norm();
            eL2_pin_cd2_ += pin2_error.norm();
            eL2_pin_cd5_ += pin5_error.norm();

            eLinf_pin_cd1_ += pin1_error.lpNorm<Eigen::Infinity>();
            eLinf_pin_cd2_ += pin2_error.lpNorm<Eigen::Infinity>();
            eLinf_pin_cd5_ += pin5_error.lpNorm<Eigen::Infinity>();
        }
    }

    GetParam()->writeToFile(GetParam()->instruction_outfile,
                            i_cluster, i_lg, i_pin_fd, 
                            i_pin_cd1, i_pin_cd2, i_pin_cd5, 
                            i_aba, i_pin_aba);

    GetParam()->writeToFile(GetParam()->timing_outfile,
                            t_cluster_ / num_samples_,
                            t_lg_ / num_samples_,
                            t_pin_fd_ / num_samples_,
                            t_pin_cd1_ / num_samples_,
                            t_pin_cd2_ / num_samples_,
                            t_pin_cd5_ / num_samples_,
                            t_aba_ / num_samples_,
                            t_pin_aba_ / num_samples_);

    GetParam()->writeToFile(GetParam()->errorL2_outfile,
                            eL2_cluster_ / num_samples_,
                            eL2_lg_ / num_samples_,
                            eL2_pin_fd_ / num_samples_,
                            eL2_pin_cd1_ / num_samples_,
                            eL2_pin_cd2_ / num_samples_,
                            eL2_pin_cd5_ / num_samples_,
                            eL2_aba_ / num_samples_,
                            eL2_pin_aba_ / num_samples_);

    GetParam()->writeToFile(GetParam()->errorInf_outfile,
                            eLinf_cluster_ / num_samples_,
                            eLinf_lg_ / num_samples_,
                            eLinf_pin_fd_ / num_samples_,
                            eLinf_pin_cd1_ / num_samples_,
                            eLinf_pin_cd2_ / num_samples_,
                            eLinf_pin_cd5_ / num_samples_,
                            eLinf_aba_ / num_samples_,
                            eLinf_pin_aba_ / num_samples_);
}
