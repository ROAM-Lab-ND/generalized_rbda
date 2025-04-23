#ifndef GRBDA_PINOCCHIO_HELPERS_H
#define GRBDA_PINOCCHIO_HELPERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <tinyxml2.h>
#include <console_bridge/console.h>

#include "gtest/gtest.h"

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"
#include "grbda/Utils/Utilities.h"
#include "grbda/Utils/Timer.h"

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

const std::string main_urdf_directory = SOURCE_DIRECTORY "/robot-models/";
const std::string urdf_directory = SOURCE_DIRECTORY "/Benchmarking/urdfs/";
const std::string path_to_data = SOURCE_DIRECTORY "/Benchmarking/data/";

template <typename Scalar>
struct JointMap
{
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pos;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> vel;
};

template <typename Scalar>
JointMap<Scalar> jointMap(const grbda::RigidBodyTreeModel<Scalar> &grbda_model,
                          const pinocchio::Model &pin_model)
{

    JointMap<Scalar> joint_map;
    using EigMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    joint_map.pos = EigMat::Zero(pin_model.nq, grbda_model.getNumPositions());
    joint_map.vel = EigMat::Zero(pin_model.nv, grbda_model.getNumDegreesOfFreedom());

    int i_pos = 0, i_vel = 0;
    for (const auto &name : pin_model.names)
    {
        for (const auto &node : grbda_model.rigidBodyNodes())
        {
            if (node->joint_->name() == name)
            {
                int nq_i = node->joint_->numPositions();
                int nv_i = node->joint_->numVelocities();

                // Detect floating base
                if (nq_i == 7 && nv_i == 6)
                {
                    joint_map.pos.block(i_pos, node->position_index_, 3, 3).setIdentity();
                    joint_map.pos.block(i_pos + 3, node->position_index_ + 4, 3, 3).setIdentity();
                    joint_map.pos(i_pos + 6, node->position_index_ + 3) = 1;

                    joint_map.vel.block(i_vel, node->velocity_index_ + 3, 3, 3).setIdentity();
                    joint_map.vel.block(i_vel + 3, node->velocity_index_, 3, 3).setIdentity();
                }
                else
                {
                    joint_map.pos.block(i_pos, node->position_index_, nq_i, nq_i).setIdentity();
                    joint_map.vel.block(i_vel, node->velocity_index_, nv_i, nv_i).setIdentity();
                }

                i_pos += nq_i;
                i_vel += nv_i;
            }
        }
    }

    // Check that the joint map is valid
    Eigen::MatrixXd jmap_pos_double = joint_map.pos.template cast<double>();
    Eigen::MatrixXd jmap_vel_double = joint_map.vel.template cast<double>();
    for (int i = 0; i < joint_map.pos.rows(); i++)
    {
        EXPECT_TRUE(jmap_pos_double.row(i).sum() == 1);
        EXPECT_NEAR(jmap_pos_double.row(i).norm(), 1, 1e-8);
    }
    for (int i = 0; i < joint_map.pos.cols(); i++)
    {
        EXPECT_TRUE(jmap_pos_double.col(i).sum() == 1);
        EXPECT_NEAR(jmap_pos_double.col(i).norm(), 1, 1e-8);
    }
    for (int i = 0; i < joint_map.vel.rows(); i++)
    {
        EXPECT_TRUE(jmap_vel_double.row(i).sum() == 1);
        EXPECT_NEAR(jmap_vel_double.row(i).norm(), 1, 1e-8);
    }
    for (int i = 0; i < joint_map.vel.cols(); i++)
    {
        EXPECT_TRUE(jmap_vel_double.col(i).sum() == 1);
        EXPECT_NEAR(jmap_vel_double.col(i).norm(), 1, 1e-8);
    }

    return joint_map;
}

template <typename Scalar>
using PinModel = pinocchio::ModelTpl<Scalar>;

template <typename Scalar>
using ConstraintModel = pinocchio::RigidConstraintModelTpl<Scalar>;

template <typename Scalar>
using ConstraintModelVector = PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ConstraintModel<Scalar>);

template <typename Scalar>
struct ConstrainedLinkInfo
{
    pinocchio::Model::JointIndex idx;
    pinocchio::SE3Tpl<Scalar> local_pose;
};

// TODO(@MatthewChignoli): This should actually return a vector of RigidConstraintModels (and data?)
// TODO(@MatthewChignoli): Maybe move this to a different location
// TODO(@MatthewChignoli): I cannot access the urdf::parseLoopConstraint function, so I think I just need to reimplement that manually
// It needs the pin model as well here

template <typename Scalar>
ConstrainedLinkInfo<Scalar> constrainedLinkInfoFromXml(tinyxml2::XMLElement *config,
                                                       const PinModel<Scalar> &model)
{
    ConstrainedLinkInfo<Scalar> info;

    const char *pname = config->Attribute("link");
    if (!pname)
    {
        CONSOLE_BRIDGE_logInform("no predecessor/successor link name specified for Constraint");
        return ConstrainedLinkInfo<Scalar>();
    }
    else
    {
        // Get the joint name associated with the link name
        pinocchio::Model::FrameIndex frame_idx = model.getFrameId(std::string(pname));
        info.idx = model.frames[frame_idx].parentJoint;
    }
    std::cout << "Found predecessor: " << pname << std::endl;
    std::cout << "Predecessor index: " << info.idx << std::endl;

    tinyxml2::XMLElement *origin_xml = config->FirstChildElement("origin");
    if (!origin_xml)
    {
        CONSOLE_BRIDGE_logDebug("urdfdom: Loop Constraint missing origin tag.");
        return ConstrainedLinkInfo<Scalar>();
    }
    else
    {
        const char *xyz_str = origin_xml->Attribute("xyz");
        std::istringstream iss{std::string(xyz_str)};
        Eigen::Matrix3<double> rotation = Eigen::Matrix3<double>::Identity();
        Eigen::Vector3<double> translation = Eigen::Vector3<double>::Zero();
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> translation[i]))
            {
                throw std::runtime_error("Invalid input: expected 3 doubles");
            }
        }
        std::cout << "translation: " << translation.transpose() << std::endl;
        info.local_pose = pinocchio::SE3(rotation, translation).cast<Scalar>();
    }

    return info;
}

template <typename Scalar>
ConstraintModelVector<Scalar> parseURDFFileForLoopConstraints(const std::string &path,
                                                              const PinModel<Scalar> &model)
{
    ConstraintModelVector<Scalar> constraint_models;
    pinocchio::ContactType constraint_type = pinocchio::ContactType::CONTACT_3D;
    pinocchio::ReferenceFrame ref_frame = pinocchio::ReferenceFrame::LOCAL;

    std::ifstream stream(path.c_str());
    if (!stream)
    {
        CONSOLE_BRIDGE_logError(("File " + path + " does not exist").c_str());
        return ConstraintModelVector<Scalar>();
    }

    std::string xml_str((std::istreambuf_iterator<char>(stream)),
                        std::istreambuf_iterator<char>());

    std::cout << "Testing XML string: " << xml_str << std::endl;

    tinyxml2::XMLDocument xml_doc;
    xml_doc.Parse(xml_str.c_str());
    if (xml_doc.Error())
    {
        CONSOLE_BRIDGE_logError(xml_doc.ErrorStr());
        xml_doc.ClearError();
        return ConstraintModelVector<Scalar>();
    }
    std::cout << "Parsed XML string" << std::endl;

    tinyxml2::XMLElement *robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml)
    {
        CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
        return ConstraintModelVector<Scalar>();
    }
    std::cout << "Found robot element" << std::endl;

    // Get all Loop Constraint elements
    for (tinyxml2::XMLElement *constraint_xml = robot_xml->FirstChildElement("loop"); constraint_xml; constraint_xml = constraint_xml->NextSiblingElement("loop"))
    {
        // Get Constraint Name
        const char *name = constraint_xml->Attribute("name");
        if (!name)
        {
            CONSOLE_BRIDGE_logError("unnamed constraint found");
            return ConstraintModelVector<Scalar>();
        }
        const std::string constraint_name(name);
        std::cout << "Found constraint: " << constraint_name << std::endl;

        // TODO(@MatthewChignoli): Implement this manually
        // pinocchio::Model::JointIndex pred_idx, succ_idx;
        // pinocchio::SE3Tpl<Scalar> SE3_from_pred, SE3_from_succ;
        ConstrainedLinkInfo<Scalar> pred_info, succ_info;

        // TODO(@MatthewChignoli): Maybe a way to check to make sure the helper returned something reasonable?
        tinyxml2::XMLElement *predecessor_xml = constraint_xml->FirstChildElement("predecessor");
        if (!predecessor_xml)
        {
            CONSOLE_BRIDGE_logError("Loop Constraint [%s] missing predecessor tag.",
                                    constraint_name.c_str());
            return ConstraintModelVector<Scalar>();
        }
        else
        {
            pred_info = constrainedLinkInfoFromXml(predecessor_xml, model);
        }

        tinyxml2::XMLElement *successor_xml = constraint_xml->FirstChildElement("successor");
        if (!successor_xml)
        {
            CONSOLE_BRIDGE_logError("Loop Constraint [%s] missing successor tag.",
                                    constraint_name.c_str());
            return ConstraintModelVector<Scalar>();
        }
        else
        {
            succ_info = constrainedLinkInfoFromXml(successor_xml, model);
        }

        // Print the predecessor and successor info
        std::cout << "Predecessor index: " << pred_info.idx << std::endl;
        std::cout << "Predecessor local pose: " << pred_info.local_pose.translation().transpose() << std::endl;
        std::cout << "Successor index: " << succ_info.idx << std::endl;
        std::cout << "Successor local pose: " << succ_info.local_pose.translation().transpose() << std::endl;

        ConstraintModel<Scalar> constraint(constraint_type, model,
                                           pred_info.idx, pred_info.local_pose,
                                           succ_info.idx, succ_info.local_pose,
                                           ref_frame);
        constraint.corrector.Kp.array() = 10.;
        constraint.corrector.Kd.array() = 10.;
        constraint_models.push_back(constraint);
    }
    return constraint_models;
}

#endif // GRBDA_PINOCCHIO_HELPERS_H
