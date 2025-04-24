#include "branchAndDepthHelpers.hpp"

TEST(BranchAndDepthBenchmark, NumericalValidation)
{
    ConstraintModelVector<double> test_constraints;

    std::string urdf_filename = main_urdf_directory + "four_bar.urdf";

    // Build models
    PinModel<double> model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // PinocchioADModel ad_model = model.cast<ADScalar>();
    // PinocchioADModel::Data ad_data(ad_model);
    // PinocchioADModel::Data ad_data_cd(ad_model);

    parseURDFFileForLoopConstraints(urdf_filename, model);
    printf("Branch and Depth Benchmark\n");
}

class BranchAndDepthNumericalValidation
    : public ::testing::TestWithParam<std::shared_ptr<RobotSpecification>>
{
protected:
    BranchAndDepthNumericalValidation()
    {
        prox_settings_ = PinProxSettings<Scalar>(prox_accuracy_, prox_mu_, prox_max_iter_);
    }

    using Scalar = double;
    using ModelState = grbda::ModelState<Scalar>;
    using JointState = grbda::JointState<Scalar>;
    using StatePair = std::pair<DVec<Scalar>, DVec<Scalar>>;

    const int num_samples_ = 8;
    const Scalar qdd_tol_ = 5e-5;
    const Scalar cnstr_tol_ = 1e-9;

    // TODO(@MatthewChignoli): Should we do multiple of these? Like what is a fair one to benchmark against?
    const Scalar prox_accuracy_ = 1e-12;
    const Scalar prox_mu_ = 1e-14;
    const int prox_max_iter_ = 1;
    PinProxSettings<Scalar> prox_settings_;
};

INSTANTIATE_TEST_SUITE_P(BranchAndDepthNumericalValidation, BranchAndDepthNumericalValidation,
                         ::testing::ValuesIn(GetBenchmarkRobotSpecifications()));

TEST_P(BranchAndDepthNumericalValidation, forward_dynamics)
{
    // TODO(@MatthewChignoli): Should this be done in the test class ctor?
    // Build models
    PinModel<Scalar> model;
    pinocchio::urdf::buildModel(GetParam()->urdf_filename, model);
    PinData<Scalar> data_fd(model);

    // TODO(@MatthewChignoli): What to do in the case of coupling constraints?
    PinData<Scalar> data_cd(model);
    auto pin_constraints = parseURDFFileForLoopConstraints(GetParam()->urdf_filename, model);
    ConstraintModelVector<Scalar> constraint_models = pin_constraints.models;
    ConstraintDataVector<Scalar> constraint_datas = pin_constraints.datas;
    pinocchio::initConstraintDynamics(model, data_cd, constraint_models);

    grbda::ClusterTreeModel<Scalar> cluster_tree;
    cluster_tree.buildModelFromURDF(GetParam()->urdf_filename);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<Scalar>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);

    // Extract info from models
    JointMap<Scalar> joint_map = jointMap(lgm_model, model);
    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    // Numerical validation
    for (int i = 0; i < num_samples_; i++)
    {
        // Create a random state
        ModelState model_state;
        ModelState spanning_model_state;
        using LoopConstraint = grbda::LoopConstraint::Base<Scalar>;
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
        DVec<Scalar> spanning_joint_pos = spanning_q_and_v.first;
        DVec<Scalar> spanning_joint_vel = spanning_q_and_v.second;
        cluster_tree.setState(model_state);
        lgm_model.setState(spanning_joint_pos, spanning_joint_vel);

        DVec<Scalar> pin_q(nv_span), pin_v(nv_span);
        pin_q = joint_map.pos * spanning_joint_pos;
        pin_v = joint_map.vel* spanning_joint_vel;

        // Extract loop constraints from the cluster tree
        cluster_tree.forwardKinematics();
        DMat<Scalar> K_cluster = DMat<Scalar>::Zero(0, 0);
        DVec<Scalar> k_cluster = DVec<Scalar>::Zero(0);
        DMat<Scalar> G_cluster = DMat<Scalar>::Zero(0, 0);
        DVec<Scalar> g_cluster = DVec<Scalar>::Zero(0);
        for (const auto &cluster : cluster_tree.clusters())
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
        const DMat<Scalar> K_pinocchio = K_cluster * joint_map.vel.transpose();
        const DVec<Scalar> k_pinocchio = -k_cluster;

        // Compute the forward dynamics
        DVec<Scalar> spanning_joint_tau = DVec<Scalar>::Random(nv_span);
        DVec<Scalar> tau = G_cluster.transpose() * spanning_joint_tau;

        DVec<Scalar> pin_tau(nv_span);
        pin_tau = joint_map.vel * spanning_joint_tau;

        const DVec<Scalar> ydd_cluster = cluster_tree.forwardDynamics(tau);
        DVec<Scalar> qdd_cluster = DVec<Scalar>::Zero(0);
        for (const auto &cluster : cluster_tree.clusters())
        {
            DVec<Scalar> ydd_k = ydd_cluster.segment(cluster->velocity_index_,
                                                        cluster->num_velocities_);
            DVec<Scalar> qdd_k = cluster->joint_->G() * ydd_k + cluster->joint_->g();
            qdd_cluster = grbda::appendEigenVector(qdd_cluster, qdd_k);
        }

        const DVec<Scalar> qdd_lgm = lgm_model.forwardDynamics(tau);

        pinocchio::forwardDynamics(model, data_fd, pin_q, pin_v, pin_tau,
                                   K_pinocchio, k_pinocchio, prox_mu_);

        pinocchio::constraintDynamics(model, data_cd, pin_q, pin_v, pin_tau,
                                      constraint_models, constraint_datas, prox_settings_);

        // Check grbda cluster tree solution against lagrange multiplier solution
        const DVec<Scalar> grbda_error = qdd_lgm - qdd_cluster;
        GTEST_ASSERT_LT(grbda_error.norm(), qdd_tol_)
            << "qdd_lgm: " << qdd_lgm.transpose() << "\n"
            << "qdd_cluster: " << qdd_cluster.transpose();

        // Check grbda lagrange multiplier solution against pinocchio
        const DVec<Scalar> pin_error = data_fd.ddq - joint_map.vel * qdd_lgm;
        GTEST_ASSERT_LT(pin_error.norm(), qdd_tol_)
            << "qdd_pinocchio   : " << data_fd.ddq.transpose() << "\n"
            << "jmap * qdd_lgm: " << (joint_map.vel * qdd_lgm).transpose();

        // Check the pinocchio constrained dynamics solution against the pinocchio forward dynamics
        const DVec<Scalar> pin_error_2 = data_fd.ddq - data_cd.ddq;
        if (!constraint_models.empty())
        {
            GTEST_ASSERT_LT(pin_error_2.norm(), qdd_tol_)
                << "qdd_pinocchio_fd: " << data_fd.ddq.transpose() << "\n"
                << "qdd_pinocchio_cd: " << data_cd.ddq.transpose();
        }

        // Check that solutions satisfy the loop constraints
        Eigen::VectorXd cnstr_violation_pinocchio = K_pinocchio * data_fd.ddq + k_pinocchio;
        Eigen::VectorXd cnstr_violation_grbda = K_cluster * qdd_lgm - k_cluster;

        GTEST_ASSERT_LT(cnstr_violation_grbda.norm(), cnstr_tol_)
            << "K*qdd_lgm + k    : " << cnstr_violation_grbda.transpose();

        GTEST_ASSERT_LT(cnstr_violation_pinocchio.norm(), cnstr_tol_)
            << "K*qdd_pinocchio + k: " << cnstr_violation_pinocchio.transpose();
    }

}
