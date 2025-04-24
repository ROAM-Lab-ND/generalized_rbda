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

void RobotSpecification::registerNameFromUrdfFilename(const std::string &url)
{
    // Find the last occurrence of '/' which indicates the start of the filename
    size_t lastSlashPos = url.find_last_of('/');
    if (lastSlashPos == std::string::npos)
    {
        lastSlashPos = -1;
    }

    // Extract filename
    std::string filename = url.substr(lastSlashPos + 1);

    // Find the last occurrence of '.' which indicates the start of the file extension
    size_t dotPos = filename.find_last_of('.');
    if (dotPos != std::string::npos)
    {
        // Subtract extension
        filename = filename.substr(0, dotPos);
    }

    name = filename;
}

void RobotSpecification::openOutfile()
{
    std::string path_to_instr = path_to_data + instruction_prefix + outfile_suffix + ".csv";
    instruction_outfile.open(path_to_instr, std::ios::app);
    if (!instruction_outfile.is_open())
    {
        std::cerr << "Failed to open instruction benchmark file." << std::endl;
    }

    std::string path_to_timing = path_to_data + timing_prefix + outfile_suffix + ".csv";
    timing_outfile.open(path_to_timing, std::ios::app);
    if (!timing_outfile.is_open())
    {
        std::cerr << "Failed to open time benchmark file." << std::endl;
    }

    std::string path_to_error = path_to_data + error_prefix + outfile_suffix + ".csv";
    error_outfile.open(path_to_error, std::ios::app);
    if (!error_outfile.is_open())
    {
        std::cerr << "Failed to open error benchmark file." << std::endl;
    }
}

void RobotSpecification::closeOutfile()
{
    if (instruction_outfile.is_open())
    {
        instruction_outfile.close();
    }

    if (timing_outfile.is_open())
    {
        timing_outfile.close();
    }

    if (error_outfile.is_open())
    {
        error_outfile.close();
    }
}

void RobotSpecification::writeToFile(std::ofstream &outfile,
                                     double i_cluster, double i_lg,
                                     double i_pin_fd, double i_pin_cd1,
                                     double i_pin_cd2, double i_pin_cd5)
{
    outfile << name << ","
            << i_cluster << ","
            << i_lg << ","
            << i_pin_fd << ","
            << i_pin_cd1 << ","
            << i_pin_cd2 << ","
            << i_pin_cd5 << std::endl;
}

void BranchAndDepthSpecification::registerBranchAndDepthCountFromName(const std::string &name_)
{
    std::regex re(R"(_(\d+)_+(\d+)_?)");
    std::smatch match;

    if (std::regex_search(name_, match, re) && match.size() > 2)
    {
        branch_count = std::stoi(match.str(1));
        depth_count = std::stoi(match.str(2));
    }
}

void BranchAndDepthSpecification::writeToFile(std::ofstream &outfile,
                                              double i_cluster, double i_lg,
                                              double i_pin_fd, double i_pin_cd1,
                                              double i_pin_cd2, double i_pin_cd5)
{
    outfile << branch_count << ","
            << depth_count << ","
            << i_cluster << ","
            << i_lg << ","
            << i_pin_fd << ","
            << i_pin_cd1 << ","
            << i_pin_cd2 << ","
            << i_pin_cd5 << std::endl;
}

ParallelChainSpecification::ParallelChainSpecification(int depth_, int loop_size_, double tol_,
                                                       std::string constraint_type_)
    : RobotSpecification(urdf_directory + "parallel_chains/" +
                             constraint_type_ + "/depth" +
                             std::to_string(depth_) + "/loop_size" +
                             std::to_string(loop_size_) + ".urdf",
                         constraint_type_),
      depth(depth_), loop_size(loop_size_), tol(tol_), constraint_type(constraint_type_) {}

template <typename ModelType>
bool ParallelChainSpecification::clusterTreeProperlyFormed(const ModelType &cluster_tree) const
{
    int expected_bodies = constraint_type == "Explicit" ? 2 * depth
                                                        : 2 * depth + 1;
    if (cluster_tree.getNumBodies() != expected_bodies)
    {
        return false;
    }

    int largest_loop_size = -1;
    for (const auto &cluster : cluster_tree.clusters())
    {
        int cur_loop_size = cluster->bodies_.size();
        if (cur_loop_size > largest_loop_size)
            largest_loop_size = cur_loop_size;
    }
    if (largest_loop_size != loop_size)
    {
        return false;
    }

    return true;
}

template bool ParallelChainSpecification::clusterTreeProperlyFormed(
    const grbda::ClusterTreeModel<double> &cluster_tree) const;
template bool ParallelChainSpecification::clusterTreeProperlyFormed(
    const grbda::ClusterTreeModel<casadi::SX> &cluster_tree) const;

void ParallelChainSpecification::writeToFile(std::ofstream &outfile,
                                             double i_cluster, double i_lg,
                                             double i_pin_fd, double i_pin_cd1,
                                             double i_pin_cd2, double i_pin_cd5)
{
    outfile << depth << ","
            << loop_size << ","
            << i_cluster << ","
            << i_lg << ","
            << i_pin_fd << ","
            << i_pin_cd1 << ","
            << i_pin_cd2 << ","
            << i_pin_cd5 << std::endl;
}

SpecificationVector GetIndividualRobotSpecifications()
{
    SpecificationVector robot_specifications;
    std::string outfile_suffix = "system";

    const std::string four_bar_urdf = main_urdf_directory + "four_bar.urdf";
    auto four_bar_spec = std::make_shared<RobotSpecification>(four_bar_urdf,
                                                              outfile_suffix);
    robot_specifications.push_back(four_bar_spec);

    const std::string rev_rotor_chain_urdf = main_urdf_directory + "revolute_rotor_chain.urdf";
    auto rev_rotor_chain_spec = std::make_shared<RobotSpecification>(rev_rotor_chain_urdf,
                                                                     outfile_suffix);
    robot_specifications.push_back(rev_rotor_chain_spec);

    const std::string mit_humanoid_urdf = main_urdf_directory + "mit_humanoid.urdf";
    auto mit_humanoid_spec = std::make_shared<RobotSpecification>(mit_humanoid_urdf,
                                                                  outfile_suffix);
    robot_specifications.push_back(mit_humanoid_spec);

    const std::string mini_cheetah_urdf = main_urdf_directory + "mini_cheetah.urdf";
    auto mini_cheetah_spec = std::make_shared<RobotSpecification>(mini_cheetah_urdf,
                                                                  outfile_suffix);
    robot_specifications.push_back(mini_cheetah_spec);

    return robot_specifications;
}

SpecificationVector GetRevoluteRotorRobotSpecifications()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_revolute_urdf dir
    SpecificationVector robot_specifications;
    std::string outfile_suffix = "revolute_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7, 8, 9, 10})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_revolute_urdf/revolute_rotor_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            auto robot_spec = std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                            outfile_suffix);
            robot_specifications.push_back(robot_spec);
        }
    return robot_specifications;
}

SpecificationVector GetRevoluteRotorPairRobotSpecifications()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_revolute_pair_urdf dir
    SpecificationVector robot_specifications;
    std::string outfile_suffix = "revolute_pair_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_revolute_pair_urdf/revolute_rotor_pair_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            auto robot_spec = std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                            outfile_suffix);
            robot_specifications.push_back(robot_spec);
        }
    return robot_specifications;
}

SpecificationVector GetFourBarRobotSpecifications()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_four_bar_urdf dir
    SpecificationVector robot_specifications;
    std::string outfile_suffix = "four_bar_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_four_bar_urdf/four_bar_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            auto robot_spec = std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                            outfile_suffix);
            robot_specifications.push_back(robot_spec);
        }
    return robot_specifications;
}

SpecificationVector GetParallelChainSpecifications()
{
    SpecificationVector parallel_chains;
    // for (int i : {2, 4, 6, 8, 10})
    for (int i : {2, 4})
    {
        auto exp_chain = std::make_shared<ParallelChainSpecification>(5, i, 1e-8, "Explicit");
        auto imp_chain = std::make_shared<ParallelChainSpecification>(5, i + 1, 1e-4, "Implicit");
        parallel_chains.push_back(exp_chain);
        parallel_chains.push_back(imp_chain);
    }
    // for (int i : {2, 4, 8, 12, 16})
    for (int i : {2, 4})
    {
        auto exp_chain = std::make_shared<ParallelChainSpecification>(10, i, 1e-6, "Explicit");
        auto imp_chain = std::make_shared<ParallelChainSpecification>(10, i + 1, 1e-3, "Implicit");
        parallel_chains.push_back(exp_chain);
        parallel_chains.push_back(imp_chain);
    }
    // for (int i : {2, 6, 12, 20, 30})
    // {
    //     auto exp_chain = std::make_shared<ParallelChainSpecification>(20, i, 1e-2, "Explicit");
    //     auto imp_chain = std::make_shared<ParallelChainSpecification>(20, i + 1, 1e1, "Implicit");
    //     parallel_chains.push_back(exp_chain);
    //     parallel_chains.push_back(imp_chain);
    // }
    // for (int i : {2, 8, 16, 28, 40})
    // {
    //     auto exp_chain = std::make_shared<ParallelChainSpecification>(40, i, 5e0, "Explicit");
    //     auto imp_chain = std::make_shared<ParallelChainSpecification>(40, i + 1, 1e2, "Implicit");
    //     parallel_chains.push_back(exp_chain);
    //     parallel_chains.push_back(imp_chain);
    // }

    return parallel_chains;
}

SpecificationVector GetBenchmarkRobotSpecifications()
{
    // Get the actual RobotSpecifications
    SpecificationVector test_robot_specs = GetIndividualRobotSpecifications();

    SpecificationVector revrotor_robot_specs = GetRevoluteRotorRobotSpecifications();
    test_robot_specs.insert(test_robot_specs.end(), revrotor_robot_specs.begin(),
                            revrotor_robot_specs.end());

    // SpecificationVector revrotor_pair_robot_specs = GetRevoluteRotorPairRobotSpecifications();
    // test_robot_specs.insert(test_robot_specs.end(), revrotor_pair_robot_specs.begin(),
    //                         revrotor_pair_robot_specs.end());

    // SpecificationVector four_bar_robot_specs = GetFourBarRobotSpecifications();
    // test_robot_specs.insert(test_robot_specs.end(), four_bar_robot_specs.begin(),
    //                         four_bar_robot_specs.end());

    SpecificationVector parallel_chain_robot_specs = GetParallelChainSpecifications();
    test_robot_specs.insert(test_robot_specs.end(), parallel_chain_robot_specs.begin(),
                            parallel_chain_robot_specs.end());

    // Test that all output files are created
    std::vector<std::string> sys_types;
    for (const auto &spec : test_robot_specs)
    {
        if (std::find(sys_types.begin(), sys_types.end(), spec->outfile_suffix) == sys_types.end())
        {
            sys_types.push_back(spec->outfile_suffix);
        }
    }

    for (std::string bench_type : {RobotSpecification::instruction_prefix,
                                   RobotSpecification::timing_prefix,
                                   RobotSpecification::error_prefix})
    {
        for (std::string sys_type : sys_types)
        {
            std::ofstream outfile;
            outfile.open(path_to_data + bench_type + sys_type + ".csv",
                         std::ios::trunc);
            if (!outfile.is_open())
            {
                std::cerr << "Failed to open file." << std::endl;
            }
            if (outfile.is_open())
            {
                outfile.close();
            }
        }
    }

    return test_robot_specs;
}
