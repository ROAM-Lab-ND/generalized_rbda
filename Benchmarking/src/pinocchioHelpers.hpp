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

#endif // GRBDA_PINOCCHIO_HELPERS_H
