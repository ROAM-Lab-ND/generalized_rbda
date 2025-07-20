#ifndef GRBDA_PINOCCHIO_HELPERS_H
#define GRBDA_PINOCCHIO_HELPERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <algorithm>
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
using DVec = Eigen::Vector<Scalar, Eigen::Dynamic>;

template <typename Scalar>
using DMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar>
struct JointMap
{
    DMat<Scalar> pos;
    DMat<Scalar> vel;
};

template <typename Scalar>
JointMap<Scalar> jointMap(const grbda::RigidBodyTreeModel<Scalar> &grbda_model,
                          const pinocchio::Model &pin_model);

template <typename Scalar>
using PinModel = pinocchio::ModelTpl<Scalar>;

template <typename Scalar>
using PinData = pinocchio::DataTpl<Scalar>;

template <typename Scalar>
using ConstraintModel = pinocchio::RigidConstraintModelTpl<Scalar>;

template <typename Scalar>
using ConstraintData = pinocchio::RigidConstraintDataTpl<Scalar>;

template <typename Scalar>
using ConstraintModelVector = PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ConstraintModel<Scalar>);

template <typename Scalar>
using ConstraintDataVector = PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ConstraintData<Scalar>);

template <typename Scalar>
struct PinConstraintVectors
{
    ConstraintModelVector<Scalar> models;
    ConstraintDataVector<Scalar> datas;
};

template <typename Scalar>
using PinProxSettings = pinocchio::ProximalSettingsTpl<Scalar>;

template <typename Scalar>
struct ConstrainedLinkInfo
{
    pinocchio::Model::JointIndex idx;
    pinocchio::SE3Tpl<Scalar> local_pose;
};

template <typename Scalar>
ConstrainedLinkInfo<Scalar> constrainedLinkInfoFromXml(tinyxml2::XMLElement *config,
                                                       const PinModel<Scalar> &model);

template <typename Scalar>
PinConstraintVectors<Scalar> parseURDFFileForLoopConstraints(const std::string &path,
                                                             const PinModel<Scalar> &model);

struct RobotSpecification
{
    std::string urdf_filename;
    std::string approx_urdf_filename;
    std::string outfile_suffix;
    static inline const std::string instruction_prefix = "InstructionPinocchioFD_";
    static inline const std::string timing_prefix = "TimingPinocchioFD_";
    static inline const std::string errorL2_prefix = "ErrorL2PinocchioFD_";
    static inline const std::string errorLinf_prefix = "ErrorLinfPinocchioFD_";
    std::ofstream instruction_outfile;
    std::ofstream timing_outfile;
    std::ofstream errorL2_outfile;
    std::ofstream errorInf_outfile;
    std::string name;

    RobotSpecification(std::string urdf_filename_,
                       std::string approx_urdf_filename_,
                       std::string outfile_suffix_)
        : urdf_filename(urdf_filename_), approx_urdf_filename(approx_urdf_filename_),
          outfile_suffix(outfile_suffix_)
    {
        registerNameFromUrdfFilename(urdf_filename);
        openOutfile();
    }

    virtual ~RobotSpecification() { closeOutfile(); }

    void registerNameFromUrdfFilename(const std::string &url);

    void openOutfile();
    void closeOutfile();

    virtual void writeToFile(std::ofstream &outfile,
                             double i_cluster, double i_lg,
                             double i_pin_fd, double i_pin_cd1,
                             double i_pin_cd2, double i_pin_cd5,
                             double i_aba, double i_pin_aba);

    void setTolerances(double qdd_tol, double qdd_cd_tol, double cnstr_tol);
    void setQddTolerance(double qdd_tol);
    void setQddCdTolerance(double qdd_cd_tol);
    void setCnstrTolerance(double cnstr_tol);
    double qdd_tol_ = 5e-5;    // For comparing qdd
    double qdd_cd_tol_ = 5e-2; // For comparing qdd with constrained dynamics
    double cnstr_tol_ = 1e-6;  // For comparing constraint violations
};

struct BranchAndDepthSpecification : public RobotSpecification
{
    int branch_count;
    int depth_count;

    BranchAndDepthSpecification(std::string urdf_filename_, std::string approx_urdf_filename_,
                                std::string outfile_suffix_)
        : RobotSpecification(urdf_filename_, approx_urdf_filename_, outfile_suffix_)
    {
        registerBranchAndDepthCountFromName(name);
    }

    void registerBranchAndDepthCountFromName(const std::string &name_);

    void writeToFile(std::ofstream &outfile,
                     double i_cluster, double i_lg,
                     double i_pin_fd, double i_pin_cd1, 
                     double i_pin_cd2, double i_pin_cd5,
                     double i_aba, double i_pin_aba) override;
};

struct ParallelChainSpecification : public RobotSpecification
{
    const int depth;
    const int loop_size;
    const std::string constraint_type;

    ParallelChainSpecification(int depth_, int loop_size_, double tol_,
                               std::string constraint_type_);

    template <typename ModelType>
    bool clusterTreeProperlyFormed(const ModelType &cluster_tree) const;

    void writeToFile(std::ofstream &outfile,
                     double i_cluster, double i_lg,
                     double i_pin_fd, double i_pin_cd1,
                     double i_pin_cd2, double i_pin_cd5,
                     double i_aba, double i_pin_aba) override;
};

using SpecificationVector = std::vector<std::shared_ptr<RobotSpecification>>;
SpecificationVector GetIndividualRobotSpecifications();
SpecificationVector GetRevoluteRotorRobotSpecifications();
SpecificationVector GetRevoluteRotorPairRobotSpecifications();
SpecificationVector GetFourBarRobotSpecifications();
SpecificationVector GetParallelChainSpecifications();
SpecificationVector GetBenchmarkRobotSpecifications();

#endif // GRBDA_PINOCCHIO_HELPERS_H
