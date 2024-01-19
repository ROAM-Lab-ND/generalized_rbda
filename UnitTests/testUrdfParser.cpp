#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"
#include "grbda/Utils/UrdfParserCompatibility.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

using namespace grbda;

void appendClusterFromUrdfCluster(std::shared_ptr<dynacore::urdf::Cluster> cluster,
                                  ClusterTreeModel<double> &cluster_model)
{
    std::vector<Body<double>> bodies;
    std::vector<JointPtr<double>> joints;
    for (std::shared_ptr<const dynacore::urdf::Link> link : cluster->links)
    {
        const std::string name = link->name;
        const std::string parent_name = link->getParent()->name;
        const SpatialInertia<double> inertia = urdfInertialToSpatialInertia(link->inertial);

        dynacore::urdf::Pose pose = link->parent_joint->parent_to_joint_origin_transform;
        spatial::Transform<double> xtree(urdfRotationToRotationMatrix(pose.rotation),
                                         urdfVector3ToVec3(pose.position));

        bodies.push_back(cluster_model.registerBody(name, inertia, parent_name, xtree));

        // TODO(@MatthewChignoli): Currently assumes all joints are revolute
        ori::CoordinateAxis axis = urdfAxisToCoordinateAxis(link->parent_joint->axis);
        joints.push_back(std::make_shared<Joints::Revolute<double>>(axis));
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
        Body<double> nca_body = cluster_model.body(nca->name);

        std::vector<Body<double>> nca_to_parent_subtree, nca_to_child_subtree;
        for (std::shared_ptr<dynacore::urdf::Link> link : constraint->nca_to_parent_subtree)
        {
            nca_to_parent_subtree.push_back(cluster_model.body(link->name));
        }
        for (std::shared_ptr<dynacore::urdf::Link> link : constraint->nca_to_child_subtree)
        {
            nca_to_child_subtree.push_back(cluster_model.body(link->name));
        }

        // Print out all of this info about ncas and subtrees
        std::cout << "\nnca_body: " << nca_body.name_ << std::endl;
        std::cout << "nca_to_parent_subtree: ";
        for (Body<double> body : nca_to_parent_subtree)
        {
            std::cout << body.name_ << " ";
        }
        std::cout << std::endl;
        std::cout << "nca_to_child_subtree: ";
        for (Body<double> body : nca_to_child_subtree)
        {
            std::cout << body.name_ << " ";
        }

        // TODO(@MatthewChignoli): Now we create the lambda function for phi
        // TODO(@MatthewChignoli): How do we know what the output dimension should be?
        phi = [](const JointCoordinate<casadi::SX> &q)
        {
            return DVec<casadi::SX>::Zero(1);
        };
    }

    // TODO(@MatthewChignoli): This is the hardest part. Turning the constraint joint into a loop constraint. I think I need to use lambda functions. And then I can use autodiff to get G and K? Or supply it manually
    // TODO(@MatthewChignoli): How do we know which coordinates are independent? Should come from the URDF
    std::vector<bool> independent_coordinates{true, false};
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
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", true);

    // Create ClusterTreeModel
    ClusterTreeModel<double> cluster_model;

    // Add floating base
    ConstLinkPtr root = model->getRoot();
    ClusterPtr root_cluster = model->getClusterContaining(root->name);
    for (ConstLinkPtr link : root_cluster->links)
    {
        const std::string name = link->name;
        const std::string parent_name = "ground";
        const SpatialInertia<double> inertia = urdfInertialToSpatialInertia(link->inertial);
        const spatial::Transform xtree = spatial::Transform<double>{};

        using Free = ClusterJoints::Free<double, ori_representation::Quaternion>;
        cluster_model.template appendBody<Free>(name, inertia, parent_name, xtree);
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
