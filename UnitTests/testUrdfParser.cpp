#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"
#include "grbda/Utils/UrdfParserCompatibility.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

void appendClusterFromUrdfCluster(std::shared_ptr<dynacore::urdf::Cluster> cluster,
                                  ClusterTreeModel<double> &cluster_model)
{
    // TODO(@MatthewChignoli): It's kind of gross that we need both double and SX versions of everything. Is there any way to avoid this?
    std::vector<Body<double>> bodies;
    std::vector<JointPtr<double>> joints;
    std::vector<Body<casadi::SX>> bodies_sx;
    std::vector<JointPtr<casadi::SX>> joints_sx;
    for (std::shared_ptr<const dynacore::urdf::Link> link : cluster->links)
    {
        const std::string name = link->name;
        const std::string parent_name = link->getParent()->name;
        const SpatialInertia<double> inertia = urdfInertialToSpatialInertia<double>(link->inertial);
        SpatialInertia<casadi::SX> inertia_sx = urdfInertialToSpatialInertia<casadi::SX>(link->inertial);

        dynacore::urdf::Pose pose = link->parent_joint->parent_to_joint_origin_transform;
        spatial::Transform<double> xtree(urdfRotationToRotationMatrix<double>(pose.rotation),
                                         urdfVector3ToVec3<double>(pose.position));
        spatial::Transform<casadi::SX> xtree_sx(urdfRotationToRotationMatrix<casadi::SX>(pose.rotation),
                                                urdfVector3ToVec3<casadi::SX>(pose.position));

        // TODO(@MatthewChignoli): The order here matters because we need to register the parent body before we can register the child body.
        // The issue now is that due to how the map stores links, they are ordered alphabetically.
        // So for now, the fix is to just make sure that the parent link is alphabetically before the child link, but we need a better solution in the future
        bodies.push_back(cluster_model.registerBody(name, inertia, parent_name, xtree));
        bodies_sx.emplace_back(bodies.back().index_, name, bodies.back().parent_index_, xtree_sx, inertia_sx, bodies.back().sub_index_within_cluster_, bodies.back().cluster_ancestor_index_, bodies.back().cluster_ancestor_sub_index_within_cluster_);

        // TODO(@MatthewChignoli): Currently assumes all joints are revolute
        ori::CoordinateAxis axis = urdfAxisToCoordinateAxis(link->parent_joint->axis);
        joints.push_back(std::make_shared<Joints::Revolute<double>>(axis));
        joints_sx.push_back(std::make_shared<Joints::Revolute<casadi::SX>>(axis));
    }

    // TODO(@MatthewChignoli): Now we deal with the constraint joints. At this point, there should only be one per cluster, but we will generalize this later
    if (cluster->constraint_joints.size() != 1)
    {
        throw std::runtime_error("There should be exactly one constraint joint per cluster");
    }

    std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)> phi;
    for (std::shared_ptr<const dynacore::urdf::ConstraintJoint> constraint : cluster->constraint_joints)
    {
        // Nearest common ancester
        std::shared_ptr<dynacore::urdf::Link> nca = constraint->nearest_common_ancestor;

        // Find the corresponding body in the cluster tree model
        // TODO(@MatthewChignoli): Do we really need this? I think now, which means it does not have to be a member of the constraint joint class
        Body<double> nca_body = cluster_model.body(nca->name);

        std::vector<Body<casadi::SX>> nca_to_parent_subtree, nca_to_child_subtree;
        for (std::shared_ptr<dynacore::urdf::Link> link : constraint->nca_to_parent_subtree)
        {
            const auto &body = cluster_model.body(link->name);
            nca_to_parent_subtree.push_back(bodies_sx[body.sub_index_within_cluster_]);
        }
        for (std::shared_ptr<dynacore::urdf::Link> link : constraint->nca_to_child_subtree)
        {
            const auto &body = cluster_model.body(link->name);
            nca_to_child_subtree.push_back(bodies_sx[body.sub_index_within_cluster_]);
        }

        // Print out all of this info about ncas and subtrees
        std::cout << "\nnca_body: " << nca_body.name_ << std::endl;
        std::cout << "nca_to_parent_subtree: ";
        for (Body<casadi::SX> body : nca_to_parent_subtree)
        {
            std::cout << body.name_ << ", ";
        }
        std::cout << std::endl;
        std::cout << "nca_to_child_subtree: ";
        for (Body<casadi::SX> body : nca_to_child_subtree)
        {
            std::cout << body.name_ << ", ";
        }
        std::cout << std::endl;

        // TODO(@MatthewChignoli): Now we create the lambda function for phi
        // TODO(@MatthewChignoli): Will things in the lambda function do out of scope if we pass with reference?
        // TODO(@MatthewChignoli): How do we know what the output dimension should be?
        phi = [nca_to_parent_subtree, nca_to_child_subtree, constraint, joints_sx](const JointCoordinate<casadi::SX> &q)
        {
            using SX = casadi::SX;

            // TODO(@MatthewChignoli): All of this stuff needs to be symbolic, might need to make some conversion functions...
            // So for the symbolics, we need to pass q through the update kinematics functions for the joints to get the joint transforms, which means that we need symbolic versions of the joints. But that should actually be pretty easy

            // Update kinematics
            for (const JointPtr<SX> joint : joints_sx)
            {
                joint->updateKinematics(q, DVec<SX>::Zero(joint->numVelocities()));
            }

            // TODO(@MatthewChignoli): Make this recursive
            using RotMat = Mat3<SX>;
            RotMat R_nca_to_nca = RotMat::Identity();

            RotMat R_nca_to_driver = (joints_sx[0]->XJ() * nca_to_parent_subtree[0].Xtree_).getRotation() * R_nca_to_nca;
            RotMat R_driver_to_nca = R_nca_to_driver.transpose();

            RotMat R_nca_to_foot = (joints_sx[1]->XJ() * nca_to_parent_subtree[1].Xtree_).getRotation() * R_driver_to_nca;
            RotMat R_foot_to_nca = R_nca_to_foot.transpose();

            RotMat R_nca_to_support = (joints_sx[2]->XJ() * nca_to_child_subtree[0].Xtree_).getRotation() * R_nca_to_nca;
            RotMat R_support_to_nca = R_nca_to_support.transpose();

            // Manual method
            // TODO(@MatthewChignoli): Make this more general and recursive
            // TODO(@MatthewChignoli): Add the spatial transforms to get everything in the NCA frame
            Vec3<SX> r_nca_to_driver = nca_to_parent_subtree[0].Xtree_.getTranslation();
            Vec3<SX> r_driver_to_foot = nca_to_parent_subtree[1].Xtree_.getTranslation();
            Vec3<SX> r_foot_to_constraint = urdfVector3ToVec3<SX>(constraint->parent_to_joint_origin_transform.position);
            Vec3<SX> r_nca_to_constraint_through_parent = R_nca_to_nca * r_nca_to_driver +
                                                          R_driver_to_nca * r_driver_to_foot +
                                                          R_foot_to_nca * r_foot_to_constraint;

            Vec3<SX> r_nca_to_support = nca_to_child_subtree[0].Xtree_.getTranslation();
            Vec3<SX> r_support_to_constraint =
                urdfVector3ToVec3<SX>(constraint->child_to_joint_origin_transform.position);
            Vec3<SX> r_nca_to_constraint_through_child = R_nca_to_nca * r_nca_to_support +
                                                         R_support_to_nca * r_support_to_constraint;

            Vec3<SX> r_constraint = r_nca_to_constraint_through_parent -
                                    r_nca_to_constraint_through_child;

            // TODO(@MatthewChignoli): Once we figured out the symbolic stuff, we can return the elements of the vec3 excluding the non-zero element of constraint->axis
            // But for now, assume axis = 0 0 1

            return DVec<SX>(r_constraint.head<2>());
        };
    }

    // TODO(@MatthewChignoli): This is the hardest part. Turning the constraint joint into a loop constraint. I think I need to use lambda functions. And then I can use autodiff to get G and K? Or supply it manually
    // TODO(@MatthewChignoli): How do we know which coordinates are independent? Should come from the URDF
    std::vector<bool> independent_coordinates{true, false, false};
    std::shared_ptr<LoopConstraint::Base<double>> constraint = std::make_shared<LoopConstraint::GenericImplicit<double>>(independent_coordinates, phi);

    // TODO(@MatthewChignoli): Are there cases where we can detect specialized versions of clusters? For example, is there a way that we can detect "revolute pair with rotors" or "revolute with rotor"?
    cluster_model.appendRegisteredBodiesAsCluster<ClusterJoints::Generic<double>>(cluster->name, bodies, joints, constraint);
}

void appendClustersViaDFS(const std::string &cluster_name, std::map<std::string, bool> &visited,
                          std::shared_ptr<dynacore::urdf::Cluster> cluster,
                          ClusterTreeModel<double> &cluster_model)
{
    visited[cluster_name] = true;
    appendClusterFromUrdfCluster(cluster, cluster_model);

    for (const std::shared_ptr<dynacore::urdf::Cluster> child : cluster->child_clusters)
    {
        const std::string &child_name = child->name;
        if (!visited[child_name])
        {
            appendClustersViaDFS(child_name, visited, child, cluster_model);
        }
    }
}

// TODO(@MatthewChignoli): Should this actually be a constructor?
// TODO(@MatthewChignoli): Assumes we are importing floating base model where root link is the floating base. Eventually generalize this to any model
ClusterTreeModel<double> clusterModelFromUrdfModel()
{
    using ConstLinkPtr = std::shared_ptr<const dynacore::urdf::Link>;
    using ClusterPtr = std::shared_ptr<dynacore::urdf::Cluster>;

    // Create URDF model
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    // model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", true);
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/planar_leg_linkage.urdf", true);

    // Create ClusterTreeModel
    ClusterTreeModel<double> cluster_model;

    // Add floating base
    ConstLinkPtr root = model->getRoot();
    ClusterPtr root_cluster = model->getClusterContaining(root->name);
    for (ConstLinkPtr link : root_cluster->links)
    {
        const std::string name = link->name;
        const std::string parent_name = "ground";
        const SpatialInertia<double> inertia = urdfInertialToSpatialInertia<double>(link->inertial);
        const spatial::Transform xtree = spatial::Transform<double>{};

        // using Free = ClusterJoints::Free<double, ori_representation::Quaternion>;
        // cluster_model.template appendBody<Free>(name, inertia, parent_name, xtree);

        using Revolute = ClusterJoints::Revolute<double>;
        cluster_model.template appendBody<Revolute>(name, inertia, parent_name, xtree, ori::CoordinateAxis::Z);
    }

    // Add remaining bodies
    std::map<std::string, bool> visited;
    for (ClusterPtr child : root_cluster->child_clusters)
    {
        appendClustersViaDFS(child->name, visited, child, cluster_model);
    }

    return cluster_model;
}

GTEST_TEST(UrdfParser, parseFile)
{
    ClusterTreeModel<double> cluster_model = clusterModelFromUrdfModel();
    cluster_model.print();
    // TODO(@MatthewChignoli): We should have this test for many different URDFs. In each case we should compare the actual number of clusters to the expected number of clusters. Likewise with the actual number of joints and expected number of joints
}

class URDFvsManualTests : public ::testing::Test
{
protected:
    URDFvsManualTests()
        : urdf_model(clusterModelFromUrdfModel()), manual_model(robot.buildClusterTreeModel()) {}

    void initializeRandomStates()
    {
        model_state.clear();
        DVec<double> spanning_joint_pos = DVec<double>::Zero(0);
        DVec<double> spanning_joint_vel = DVec<double>::Zero(0);
        for (const auto &cluster : manual_model.clusters())
        {
            JointState<> joint_state = cluster->joint_->randomJointState();
            JointState<> spanning_joint_state = cluster->joint_->toSpanningTreeState(joint_state);

            spanning_joint_pos = appendEigenVector(spanning_joint_pos,
                                                   spanning_joint_state.position);
            spanning_joint_vel = appendEigenVector(spanning_joint_vel,
                                                   spanning_joint_state.velocity);
            model_state.push_back(joint_state);
        }

        urdf_model.setState(model_state);
        manual_model.setState(model_state);
    }

    PlanarLegLinkage<double> robot;
    ClusterTreeModel<double> urdf_model;
    ClusterTreeModel<double> manual_model;

    ModelState<double> model_state;
};

static const double tol = 1e-10;

// TODO(@MatthewChignoli): This is basicaly a direct copt of the testForwardKinematics unit test. Any way to reduce code duplication?
TEST_F(URDFvsManualTests, compareToManuallyConstructed)
{
    std::cout << "URDF model:" << std::endl;
    this->initializeRandomStates();

    // Verify link kinematics
    for (const auto &body : this->manual_model.bodies())
    {
        std::cout << "Body: " << body.name_ << std::endl;
        const Vec3<double> p_manual = this->manual_model.getPosition(body.name_);
        const Vec3<double> p_urdf = this->urdf_model.getPosition(body.name_);
        GTEST_ASSERT_LT((p_manual - p_urdf).norm(), tol);

        const Mat3<double> R_manual = this->manual_model.getOrientation(body.name_);
        const Mat3<double> R_urdf = this->urdf_model.getOrientation(body.name_);
        GTEST_ASSERT_LT((R_manual - R_urdf).norm(), tol);

        const Vec3<double> v_manual = this->manual_model.getLinearVelocity(body.name_);
        const Vec3<double> v_urdf = this->urdf_model.getLinearVelocity(body.name_);
        GTEST_ASSERT_LT((v_manual - v_urdf).norm(), tol);

        const Vec3<double> w_manual = this->manual_model.getAngularVelocity(body.name_);
        const Vec3<double> w_urdf = this->urdf_model.getAngularVelocity(body.name_);
        GTEST_ASSERT_LT((w_manual - w_urdf).norm(), tol);
    }
}
