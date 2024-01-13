#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"
#include "grbda/Utils/UrdfParserCompatibility.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

using namespace grbda;

// TODO(@MatthewChignoli): Change this name to denote it deals with adding to tree
void dfs_first_pass(const std::string &cluster_name, std::map<std::string, bool> &visited,
                    std::shared_ptr<dynacore::urdf::Cluster> cluster,
                    ClusterTreeModel<double> &cluster_model)
{
    using ConstLinkPtr = std::shared_ptr<const dynacore::urdf::Link>;
    using ClusterPtr = std::shared_ptr<dynacore::urdf::Cluster>;

    visited[cluster_name] = true;

    std::vector<Body<double>> bodies;
    std::vector<JointPtr<double>> joints;
    for (ConstLinkPtr link : cluster->links)
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

    // TODO(@MatthewChignoli): This is the hardest part. Turning the constraint joint into a loop constraint. I think I need to use lambda functions. And then I can use autodiff to get G and K? Or supply it manually
    std::shared_ptr<LoopConstraint::Base<double>> constraint = std::make_shared<LoopConstraint::Static<double>>(DMat<double>::Zero(0, 0), DMat<double>::Zero(0, 0));

    // TODO(@MatthewChignoli): Are there cases where we can detect specialized versions of clusters? For example, is there a way that we can detect "revolute pair with rotors" or "revolute with rotor"?
    cluster_model.appendRegisteredBodiesAsCluster<ClusterJoints::Generic<double>>(cluster->name, bodies, joints, constraint);

    for (const ClusterPtr child : cluster->child_clusters)
    {
        const std::string &child_name = child->name;
        if (!visited[child_name])
        {
            dfs_first_pass(child_name, visited, child, cluster_model);
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
        dfs_first_pass(child->name, visited, child, cluster_model);
    }

    return cluster_model;
}

GTEST_TEST(UrdfParser, parseFile)
{
    ClusterTreeModel<double> cluster_model = clusterModelFromUrdfModel();
    cluster_model.print();
}
