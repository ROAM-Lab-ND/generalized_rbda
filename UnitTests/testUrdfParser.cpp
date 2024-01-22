#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"
#include "grbda/Utils/UrdfParserCompatibility.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Robots/RobotTypes.h"

using namespace grbda;

// TODO(@MatthewChignoli): This should be a member function of ClusterTreeModel?
void appendClusterFromUrdfCluster(std::shared_ptr<dynacore::urdf::Cluster> cluster,
                                  ClusterTreeModel<double> &cluster_model)
{
    using SX = casadi::SX;

    // TODO(@MatthewChignoli): It's kind of gross that we need both double and SX versions of everything. Is there any way to avoid this?
    // TODO(@MatthewChignoli): I think order matters here? So the joints should actually be a std::map. Which means we need to chante the constructor of the GenericCluster
    // TODO(@MatthewChignoli): Yeah this needs clean up for sure
    std::vector<Body<double>> bodies;
    std::vector<JointPtr<double>> joints_vec;
    std::map<std::string, JointPtr<double>> joints;
    std::vector<Body<SX>> bodies_sx;
    std::map<std::string, JointPtr<SX>> joints_sx;
    for (std::shared_ptr<const dynacore::urdf::Link> link : cluster->links)
    {
        const std::string name = link->name;
        std::cout << "Registering body: " << name << std::endl;
        const std::string parent_name = link->getParent()->name;
        const SpatialInertia<double> inertia = urdfInertialToSpatialInertia<double>(link->inertial);
        SpatialInertia<SX> inertia_sx = urdfInertialToSpatialInertia<SX>(link->inertial);

        dynacore::urdf::Pose pose = link->parent_joint->parent_to_joint_origin_transform;
        spatial::Transform<double> xtree(urdfRotationToRotationMatrix<double>(pose.rotation),
                                         urdfVector3ToVec3<double>(pose.position));
        spatial::Transform<SX> xtree_sx(urdfRotationToRotationMatrix<SX>(pose.rotation),
                                        urdfVector3ToVec3<SX>(pose.position));

        // TODO(@MatthewChignoli): The order here matters because we need to register the parent body before we can register the child body.
        // The issue now is that due to how the map stores links, they are ordered alphabetically.
        // So for now, the fix is to just make sure that the parent link is alphabetically before the child link, but we need a better solution in the future
        bodies.push_back(cluster_model.registerBody(name, inertia, parent_name, xtree));
        bodies_sx.emplace_back(bodies.back().index_, name, bodies.back().parent_index_, xtree_sx, inertia_sx, bodies.back().sub_index_within_cluster_, bodies.back().cluster_ancestor_index_, bodies.back().cluster_ancestor_sub_index_within_cluster_);

        // TODO(@MatthewChignoli): Currently assumes all joints are revolute
        ori::CoordinateAxis axis = urdfAxisToCoordinateAxis(link->parent_joint->axis);
        joints_vec.push_back(std::make_shared<Joints::Revolute<double>>(axis));
        // joints_sx.push_back(std::make_shared<Joints::Revolute<casadi::SX>>(axis));
        joints.insert({name, std::make_shared<Joints::Revolute<double>>(axis)});
        joints_sx.insert({name, std::make_shared<Joints::Revolute<SX>>(axis)});
    }

    // TODO(@MatthewChignoli): At this point, there should only be one constraint joint per cluster, but we will generalize this later
    if (cluster->constraint_joints.size() != 1)
    {
        throw std::runtime_error("There should be exactly one constraint joint per cluster");
    }

    std::function<DVec<SX>(const JointCoordinate<SX> &)> phi;
    for (std::shared_ptr<const dynacore::urdf::ConstraintJoint> constraint : cluster->constraint_joints)
    {
        std::vector<Body<SX>> nca_to_parent_subtree, nca_to_child_subtree;
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

        // TODO(@MatthewChignoli): Will things in the lambda function do out of scope if we pass with reference?
        // TODO(@MatthewChignoli): How do we know what the output dimension should be?
        phi = [nca_to_parent_subtree, nca_to_child_subtree, constraint, joints_sx](const JointCoordinate<SX> &q)
        {
            using RotMat = Mat3<SX>;

            // Update kinematics
            int jidx = 0;
            for (const auto &joint : joints_sx)
            {
                joint.second->updateKinematics(q.segment(jidx, joint.second->numPositions()),
                                               DVec<SX>::Zero(joint.second->numVelocities()));
                jidx += joint.second->numPositions();
            }

            // Through parent
            RotMat R_to_nca = RotMat::Identity();
            Vec3<SX> r_nca_to_constraint_through_parent = Vec3<SX>::Zero();
            for (const Body<SX> &body : nca_to_parent_subtree)
            {
                const spatial::Transform<SX>& Xtree = body.Xtree_;
                const spatial::Transform<SX>& XJ = joints_sx.at(body.name_)->XJ();

                r_nca_to_constraint_through_parent += R_to_nca * Xtree.getTranslation();
                RotMat Rup = (XJ * Xtree).getRotation();
                R_to_nca = R_to_nca * Rup.transpose();
            }
            r_nca_to_constraint_through_parent +=
                R_to_nca *
                urdfVector3ToVec3<SX>(constraint->parent_to_joint_origin_transform.position);

            // Through child
            R_to_nca = RotMat::Identity();
            Vec3<SX> r_nca_to_constraint_through_child = Vec3<SX>::Zero();
            for (const Body<SX> &body : nca_to_child_subtree)
            {
                const spatial::Transform<SX>& Xtree = body.Xtree_;
                const spatial::Transform<SX>& XJ = joints_sx.at(body.name_)->XJ();

                r_nca_to_constraint_through_child += R_to_nca * Xtree.getTranslation();
                RotMat Rup = (XJ * Xtree).getRotation();
                R_to_nca = R_to_nca * Rup.transpose();
            }
            r_nca_to_constraint_through_child +=
                R_to_nca *
                urdfVector3ToVec3<SX>(constraint->child_to_joint_origin_transform.position);

            // Compute constraint
            Vec3<SX> r_constraint = r_nca_to_constraint_through_parent -
                                    r_nca_to_constraint_through_child;

            // TODO(@MatthewChignoli): Make sure unit test covers all of these cases
            Vec3<double> constraint_axis = urdfVector3ToVec3<double>(constraint->axis);
            if (constraint_axis == Vec3<double>(1, 0, 0))
            {
                return DVec<SX>(r_constraint.tail<2>());
            }
            else if (constraint_axis == Vec3<double>(0, 1, 0))
            {
                return DVec<SX>(r_constraint({0, 2}));
            }
            else if (constraint_axis == Vec3<double>(0, 0, 1))
            {
                return DVec<SX>(r_constraint.head<2>());
            }
            else
            {
                throw std::runtime_error("Constraint axis must be one of the standard axes");
            }
        };
    }

    // TODO(@MatthewChignoli): How do we know which coordinates are independent? Should come from the URDF
    std::vector<bool> independent_coordinates{true, false, false};
    std::shared_ptr<LoopConstraint::Base<double>> constraint = std::make_shared<LoopConstraint::GenericImplicit<double>>(independent_coordinates, phi);

    // TODO(@MatthewChignoli): Are there cases where we can detect specialized versions of clusters? For example, is there a way that we can detect "revolute pair with rotors" or "revolute with rotor"?
    cluster_model.appendRegisteredBodiesAsCluster<ClusterJoints::Generic<double>>(cluster->name, bodies, joints_vec, constraint);
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

// TODO(@MatthewChignoli): Should this actually be a constructor of the ClusterTreeModel class?
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

    // TODO(@MatthewChignoli): How to know whether to import as floating or fixed base? If you want fixed base, should the base be a link in the URDF?
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

// TODO(@MatthewChignoli): Add the rolling without slipping constraint to the URDF model

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

    // TODO(@MatthewChignoli): Still need to compare the dynamics
}
