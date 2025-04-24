#include "pinocchioHelpers.hpp"

template <typename Scalar>
JointMap<Scalar> jointMap(const grbda::RigidBodyTreeModel<Scalar> &grbda_model,
                          const pinocchio::Model &pin_model)
{
    JointMap<Scalar> joint_map;
    joint_map.pos = DMat<Scalar>::Zero(pin_model.nq, grbda_model.getNumPositions());
    joint_map.vel = DMat<Scalar>::Zero(pin_model.nv, grbda_model.getNumDegreesOfFreedom());

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
    DMat<double> jmap_pos_double = joint_map.pos.template cast<double>();
    DMat<double> jmap_vel_double = joint_map.vel.template cast<double>();
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

template JointMap<double> jointMap(const grbda::RigidBodyTreeModel<double> &grbda_model,
                                   const pinocchio::Model &pin_model);
template JointMap<casadi::SX> jointMap(const grbda::RigidBodyTreeModel<casadi::SX> &grbda_model,
                                   const pinocchio::Model &pin_model);

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
        info.local_pose = pinocchio::SE3(rotation, translation).cast<Scalar>();
    }

    return info;
}

template ConstrainedLinkInfo<double> constrainedLinkInfoFromXml(tinyxml2::XMLElement *config,
                                                                const PinModel<double> &model);

template <typename Scalar>
PinConstraintVectors<Scalar> parseURDFFileForLoopConstraints(const std::string &path,
                                                              const PinModel<Scalar> &model)
{
    ConstraintModelVector<Scalar> constraint_models;
    ConstraintDataVector<Scalar> constraint_datas;
    pinocchio::ContactType constraint_type = pinocchio::ContactType::CONTACT_3D;
    pinocchio::ReferenceFrame ref_frame = pinocchio::ReferenceFrame::LOCAL;

    std::ifstream stream(path.c_str());
    if (!stream)
    {
        CONSOLE_BRIDGE_logError(("File " + path + " does not exist").c_str());
        return PinConstraintVectors<Scalar>();
    }

    std::string xml_str((std::istreambuf_iterator<char>(stream)),
                        std::istreambuf_iterator<char>());

    tinyxml2::XMLDocument xml_doc;
    xml_doc.Parse(xml_str.c_str());
    if (xml_doc.Error())
    {
        CONSOLE_BRIDGE_logError(xml_doc.ErrorStr());
        xml_doc.ClearError();
        return PinConstraintVectors<Scalar>();
    }

    tinyxml2::XMLElement *robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml)
    {
        CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
        return PinConstraintVectors<Scalar>();
    }

    // Get all Loop Constraint elements
    for (tinyxml2::XMLElement *constraint_xml = robot_xml->FirstChildElement("loop"); constraint_xml; constraint_xml = constraint_xml->NextSiblingElement("loop"))
    {
        // Get Constraint Name
        const char *name = constraint_xml->Attribute("name");
        if (!name)
        {
            CONSOLE_BRIDGE_logError("unnamed constraint found");
            return PinConstraintVectors<Scalar>();
        }
        const std::string constraint_name(name);

        ConstrainedLinkInfo<Scalar> pred_info, succ_info;

        // TODO(@MatthewChignoli): Maybe a way to check to make sure the helper returned something reasonable?
        tinyxml2::XMLElement *predecessor_xml = constraint_xml->FirstChildElement("predecessor");
        if (!predecessor_xml)
        {
            CONSOLE_BRIDGE_logError("Loop Constraint [%s] missing predecessor tag.",
                                    constraint_name.c_str());
            return PinConstraintVectors<Scalar>();
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
            return PinConstraintVectors<Scalar>();
        }
        else
        {
            succ_info = constrainedLinkInfoFromXml(successor_xml, model);
        }

        // // Print the predecessor and successor info
        // std::cout << "Predecessor index: " << pred_info.idx << std::endl;
        // std::cout << "Predecessor local pose: " << pred_info.local_pose.translation().transpose() << std::endl;
        // std::cout << "Successor index: " << succ_info.idx << std::endl;
        // std::cout << "Successor local pose: " << succ_info.local_pose.translation().transpose() << std::endl;

        ConstraintModel<Scalar> constraint(constraint_type, model,
                                           pred_info.idx, pred_info.local_pose,
                                           succ_info.idx, succ_info.local_pose,
                                           ref_frame);
        constraint.corrector.Kp.array() = 10.;
        constraint.corrector.Kd.array() = 10.;
        constraint_models.push_back(constraint);
        constraint_datas.push_back(ConstraintData<Scalar>(constraint));
    }

    PinConstraintVectors<Scalar> constraints;
    constraints.models = constraint_models;
    constraints.datas = constraint_datas;
    return constraints;
}

template PinConstraintVectors<double> parseURDFFileForLoopConstraints(
    const std::string &path, const PinModel<double> &model);
template PinConstraintVectors<casadi::SX> parseURDFFileForLoopConstraints(
    const std::string &path, const PinModel<casadi::SX> &model);
