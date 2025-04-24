#include "branchAndDepthHelpers.hpp"

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
    instruction_outfile.open(path_to_data + instruction_prefix + outfile_suffix + ".csv", std::ios::app);
    if (!instruction_outfile.is_open())
    {
        std::cerr << "Failed to open instruction benchmark file." << std::endl;
    }
    timing_outfile.open(path_to_data + timing_prefix + outfile_suffix + ".csv", std::ios::app);
    if (!timing_outfile.is_open())
    {
        std::cerr << "Failed to open time benchmark file." << std::endl;
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
}

void RobotSpecification::writeToFile(std::ofstream &outfile,
                                     double i_cluster, double i_pinocchio, double i_lg)
{
    outfile << name << ","
            << i_cluster << ","
            << i_pinocchio << ","
            << i_lg << std::endl;
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
                                              double i_cluster, double i_pinocchio, double i_lg)
{
    outfile << branch_count << ","
            << depth_count << ","
            << i_cluster << ","
            << i_pinocchio << ","
            << i_lg << std::endl;
}

SpecificationVector GetIndividualRobotSpecifications()
{

    SpecificationVector robot_specifications;

    const std::string four_bar_urdf = main_urdf_directory + "four_bar.urdf";
    auto four_bar_spec = std::make_shared<RobotSpecification>(four_bar_urdf);
    robot_specifications.push_back(four_bar_spec);

    const std::string rev_rotor_chain_urdf = main_urdf_directory + "revolute_rotor_chain.urdf";
    auto rev_rotor_chain_spec = std::make_shared<RobotSpecification>(rev_rotor_chain_urdf);
    robot_specifications.push_back(rev_rotor_chain_spec);

    const std::string mit_humanoid_urdf = main_urdf_directory + "mit_humanoid.urdf";
    auto mit_humanoid_spec = std::make_shared<RobotSpecification>(mit_humanoid_urdf);
    robot_specifications.push_back(mit_humanoid_spec);

    const std::string mini_cheetah_urdf = main_urdf_directory + "mini_cheetah.urdf";
    auto mini_cheetah_spec = std::make_shared<RobotSpecification>(mini_cheetah_urdf);
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

SpecificationVector GetBenchmarkRobotSpecifications()
{
    // Test that all output files are created
    std::vector<std::string> sys_types = {"revolute_chain", "revolute_pair_chain",
                                          "four_bar_chain", "systems"};
    for (std::string bench_type : {"Instruction", "Timing"})
    {
        for (std::string sys_type : sys_types)
        {
            std::ofstream outfile;
            outfile.open(path_to_data + bench_type + "PinocchioFD_" + sys_type + ".csv",
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

    // Get the actual RobotSpecifications
    SpecificationVector test_robot_specs = GetIndividualRobotSpecifications();

    SpecificationVector revrotor_robot_specs = GetRevoluteRotorRobotSpecifications();
    test_robot_specs.insert(test_robot_specs.end(), revrotor_robot_specs.begin(),
                            revrotor_robot_specs.end());

    // SpecificationVector revrotor_pair_robot_specs = GetRevoluteRotorPairRobotSpecifications();
    // test_robot_specs.insert(test_robot_specs.end(), revrotor_pair_robot_specs.begin(),
    //                         revrotor_pair_robot_specs.end());

    SpecificationVector four_bar_robot_specs = GetFourBarRobotSpecifications();
    test_robot_specs.insert(test_robot_specs.end(), four_bar_robot_specs.begin(),
                            four_bar_robot_specs.end());

    return test_robot_specs;
}
