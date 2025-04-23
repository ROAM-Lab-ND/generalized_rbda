#ifndef GRBDA_BRANCH_AND_DEPTH_HELPERS_H
#define GRBDA_BRANCH_AND_DEPTH_HELPERS_H

#include <regex>
#include "pinocchioHelpers.hpp"

// TOOD(@MatthewChignoli): Maybe break out a source file

struct RobotSpecification
{
    std::string urdf_filename;
    std::string outfile_suffix;
    std::string instruction_prefix = "InstructionPinocchioFD_";
    std::string timing_prefix = "TimingPinocchioFD_";
    std::ofstream instruction_outfile;
    std::ofstream timing_outfile;
    std::string name;

    RobotSpecification(std::string urdf_filename_, std::string outfile_suffix_ = "systems")
        : urdf_filename(urdf_filename_), outfile_suffix(outfile_suffix_)
    {
        registerNameFromUrdfFilename(urdf_filename);
        openOutfile();
    }

    virtual ~RobotSpecification()
    {
        closeOutfile();
    }

    void registerNameFromUrdfFilename(const std::string &url)
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

    void openOutfile()
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

    void closeOutfile()
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

    virtual void writeToFile(std::ofstream &outfile, double i_cluster, double i_pinocchio, double i_lg)
    {
        outfile << name << ","
                << i_cluster << ","
                << i_pinocchio << ","
                << i_lg << std::endl;
    }
};

struct BranchAndDepthSpecification : public RobotSpecification
{
    int branch_count;
    int depth_count;

    BranchAndDepthSpecification(std::string urdf_filename_,
                                std::string outfile_suffix_ = "revolute_chain")
        : RobotSpecification(urdf_filename_, outfile_suffix_)
    {
        registerBranchAndDepthCountFromName(name);
    }

    void registerBranchAndDepthCountFromName(const std::string &name_)
    {
        std::regex re(R"(_(\d+)_+(\d+)_?)");
        std::smatch match;

        if (std::regex_search(name_, match, re) && match.size() > 2)
        {
            branch_count = std::stoi(match.str(1));
            depth_count = std::stoi(match.str(2));
        }
    }

    void writeToFile(std::ofstream &outfile, double i_cluster, double i_pinocchio, double i_lg) override
    {
        outfile << branch_count << ","
                << depth_count << ","
                << i_cluster << ","
                << i_pinocchio << ","
                << i_lg << std::endl;
    }
};

using SpecVector = std::vector<std::shared_ptr<RobotSpecification>>;

SpecVector GetIndividualUrdfFiles()
{

    SpecVector urdf_files;
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "four_bar.urdf"));
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "revolute_rotor_chain.urdf"));
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "mit_humanoid.urdf"));
    urdf_files.push_back(std::make_shared<RobotSpecification>(
        main_urdf_directory + "mini_cheetah.urdf"));
    return urdf_files;
}

SpecVector GetRevoluteRotorUrdfFiles()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_revolute_urdf dir
    SpecVector urdf_files;
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7, 8, 9, 10})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_revolute_urdf/revolute_rotor_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            urdf_files.push_back(std::make_shared<BranchAndDepthSpecification>(urdf_file));
        }
    return urdf_files;
}

SpecVector GetRevoluteRotorPairUrdfFiles()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_revolute_pair_urdf dir
    SpecVector urdf_files;
    std::string outfile_suffix = "revolute_pair_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_revolute_pair_urdf/revolute_rotor_pair_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            urdf_files.push_back(std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                               outfile_suffix));
        }
    return urdf_files;
}

SpecVector GetFourBarUrdfFiles()
{
    // TODO(@nicholasadr): automatically search for urdf files from variable_four_bar_urdf dir
    SpecVector urdf_files;
    std::string outfile_suffix = "four_bar_chain";
    for (int b : {1, 2, 4, 6})
        for (int d : {1, 2, 3, 4, 5, 6, 7})
        {
            std::string urdf_file = urdf_directory +
                                    "variable_four_bar_urdf/four_bar_branch_" +
                                    std::to_string(b) + "_" + std::to_string(d) + ".urdf";
            urdf_files.push_back(std::make_shared<BranchAndDepthSpecification>(urdf_file,
                                                                               outfile_suffix));
        }
    return urdf_files;
}

SpecVector GetBenchmarkUrdfFiles()
{
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

    SpecVector test_urdf_files = GetIndividualUrdfFiles();

    SpecVector revrotor_urdf_files = GetRevoluteRotorUrdfFiles();
    test_urdf_files.insert(test_urdf_files.end(), revrotor_urdf_files.begin(),
                           revrotor_urdf_files.end());

    SpecVector revrotor_pair_urdf_files = GetRevoluteRotorPairUrdfFiles();
    test_urdf_files.insert(test_urdf_files.end(), revrotor_pair_urdf_files.begin(),
                           revrotor_pair_urdf_files.end());

    SpecVector four_bar_urdf_files = GetFourBarUrdfFiles();
    test_urdf_files.insert(test_urdf_files.end(), four_bar_urdf_files.begin(),
                           four_bar_urdf_files.end());

    return test_urdf_files;
}

#endif // GRBDA_BRANCH_AND_DEPTH_HELPERS_H
