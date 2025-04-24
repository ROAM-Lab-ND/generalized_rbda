#ifndef GRBDA_BRANCH_AND_DEPTH_HELPERS_H
#define GRBDA_BRANCH_AND_DEPTH_HELPERS_H

#include <regex>
#include "pinocchioHelpers.hpp"

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

    void registerNameFromUrdfFilename(const std::string &url);

    void openOutfile();

    void closeOutfile();

    virtual void writeToFile(std::ofstream &outfile,
                             double i_cluster, double i_lg,
                             double i_pin_fd, double i_pin_cd1,
                             double i_pin_cd2, double i_pin_cd5);
};

struct BranchAndDepthSpecification : public RobotSpecification
{
    int branch_count;
    int depth_count;

    BranchAndDepthSpecification(std::string urdf_filename_, std::string outfile_suffix_)
        : RobotSpecification(urdf_filename_, outfile_suffix_)
    {
        registerBranchAndDepthCountFromName(name);
    }

    void registerBranchAndDepthCountFromName(const std::string &name_);

    void writeToFile(std::ofstream &outfile,
                     double i_cluster, double i_lg,
                     double i_pin_fd, double i_pin_cd1, 
                     double i_pin_cd2, double i_pin_cd5) override;
};

using SpecificationVector = std::vector<std::shared_ptr<RobotSpecification>>;
SpecificationVector GetIndividualRobotSpecifications();
SpecificationVector GetRevoluteRotorRobotSpecifications();
SpecificationVector GetRevoluteRotorPairRobotSpecifications();
SpecificationVector GetFourBarRobotSpecifications();
SpecificationVector GetBenchmarkRobotSpecifications();

#endif // GRBDA_BRANCH_AND_DEPTH_HELPERS_H
