#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

// TODO(@MatthewChignoli): Don't use this namespace. But also should make the namespace shorter
using namespace grbda;
// using namespace dynacore::urdf;

// TODO(@MatthewChignoli): Overload the Eigen::Vector<3> 

ClusterTreeModel<double> clusterModelFromUrdfModel()
{
    using LinkPtr = std::shared_ptr<dynacore::urdf::Link>;
    using ConstLinkPtr = std::shared_ptr<const dynacore::urdf::Link>;
    
    ClusterTreeModel<double> cluster_model;
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", true);

    // TODO(@MatthewChignoli): This assume floating base
    // Torso
    ConstLinkPtr root = model->getRoot();
    const std::string root_name = root->name;
    const std::string root_parent_name = "ground";
    double root_mass = root->inertial->mass;
    Vec3<double> root_COM = Vec3<double>(root->inertial->origin.position.x,
                                         root->inertial->origin.position.y,
                                         root->inertial->origin.position.z);
    Mat3<double> root_inertia;
    root_inertia.row(0) << root->inertial->ixx, root->inertial->ixy, root->inertial->ixz;
    root_inertia.row(1) << root->inertial->ixy, root->inertial->iyy, root->inertial->iyz;
    root_inertia.row(2) << root->inertial->ixz, root->inertial->iyz, root->inertial->izz;
    const SpatialInertia<double> rootInertia(root_mass, root_COM, root_inertia);

    cluster_model.template appendBody<ClusterJoints::Free<double, ori_representation::Quaternion>>(root_name, rootInertia, root_parent_name, spatial::Transform<double>{});

    // TODO(@MatthewChignoli): Add the collision box later

    // TODO(@MatthewChignoli): This should be recursive somehow
    std::vector<LinkPtr> child_links = root->child_links;
    std::vector<Body<double>> bodies;
    std::vector<JointPtr<double>> joints;
    for (LinkPtr child : child_links)
    {
        std::cout << "Child name: " << child->name << std::endl;
        ConstLinkPtr link = model->getLink(child->name);
        const std::string name = link->name;
        const std::string parent_name = link->getParent()->name;
        std::shared_ptr<dynacore::urdf::Inertial> inertial = link->inertial;
        double mass = inertial->mass;
        Vec3<double> COM = Vec3<double>(inertial->origin.position.x,
                                         inertial->origin.position.y,
                                         inertial->origin.position.z);
        Mat3<double> inertia;
        inertia.row(0) << inertial->ixx, inertial->ixy, inertial->ixz;
        inertia.row(1) << inertial->ixy, inertial->iyy, inertial->iyz;
        inertia.row(2) << inertial->ixz, inertial->iyz, inertial->izz;
        const SpatialInertia<double> Inertia(mass, COM, inertia);

        dynacore::urdf::Pose pose = link->parent_joint->parent_to_joint_origin_transform;
        Vec3<double> translation = Vec3<double>(pose.position.x, pose.position.y, pose.position.z);
        Quat<double> rotation = Quat<double>(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
        spatial::Transform<double> xtree(ori::quaternionToRotationMatrix(rotation), translation);

        bodies.push_back(cluster_model.registerBody(name, Inertia, parent_name, xtree));

        // TODO(@MatthewChignoli): Assume all revolute joints for now. But in the future we will need to check the type of joint as well as the axis. Probably with a switch statement
        // Now we need the joints
        // std::shared_ptr<dynacore::urdf::Joint> child_joint = link->parent_joint;
        joints.emplace_back(new Joints::Revolute<double>(ori::CoordinateAxis::Z));

    }



    return cluster_model;
}


GTEST_TEST(UrdfParser, parseFile)
{
    ClusterTreeModel<double> cluster_model = clusterModelFromUrdfModel();
    cluster_model.print();
}
