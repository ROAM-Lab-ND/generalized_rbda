#include "pinocchioHelpers.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"

TEST(TestConstraintDynamics, fourBar)
{
    printf("TestConstraintDynamics\n");

    // Aliases
    using ModelState = grbda::ModelState<double>;
    using JointState = grbda::JointState<double>;
    using StatePair = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
    using ConstraintModel = pinocchio::RigidConstraintModel;
    using ConstraintData = pinocchio::RigidConstraintData;
    using ConstraintType = pinocchio::ContactType;

    // URDF File
    std::string urdf_filename = main_urdf_directory + "four_bar.urdf";

    // Build Models
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // TODO(@MatthewChignoli): Better names for these?
    pinocchio::Data data(model);    // For forward dynamics
    pinocchio::Data data_cd(model); // For constrained dynamics

    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ConstraintModel)
    constraint_models;
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ConstraintData)
    constraint_datas;
    ConstraintType constraint_type = ConstraintType::CONTACT_3D;
    const pinocchio::Model::JointIndex j2 = model.getJointId("joint2");
    const pinocchio::SE3 SE3_from_joint_2 = pinocchio::SE3(Eigen::Matrix3d::Identity(),
                                                           Eigen::Vector3d(1., 0., 0.));
    const pinocchio::Model::JointIndex j3 = model.getJointId("joint3");
    const pinocchio::SE3 SE3_from_joint_3 = pinocchio::SE3(Eigen::Matrix3d::Identity(),
                                                           Eigen::Vector3d(0.5, 0., 0.));
    ConstraintModel pin_loop_constraint(constraint_type, model, j2, SE3_from_joint_2,
                                        j3, SE3_from_joint_3, pinocchio::ReferenceFrame::LOCAL);
    pin_loop_constraint.corrector.Kp.array() = 10.;
    pin_loop_constraint.corrector.Kd.array() = 10.;
    constraint_models.push_back(pin_loop_constraint);
    constraint_datas.push_back(ConstraintData(pin_loop_constraint));

    const double mu0 = 1e-14;
    pinocchio::ProximalSettings prox_settings(1e-12, mu0, 100);
    pinocchio::initConstraintDynamics(model, data_cd, constraint_models);

    grbda::ClusterTreeModel<double> cluster_tree;
    cluster_tree.buildModelFromURDF(urdf_filename);

    using RigidBodyTreeModel = grbda::RigidBodyTreeModel<double>;
    RigidBodyTreeModel lgm_model(cluster_tree, grbda::FwdDynMethod::LagrangeMultiplierEigen);

    // Extract info from models
    JointMap<double> joint_map = jointMap(lgm_model, model);
    const int nv = cluster_tree.getNumDegreesOfFreedom();
    const int nv_span = lgm_model.getNumDegreesOfFreedom();

    const double tol = 1e-12;
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
