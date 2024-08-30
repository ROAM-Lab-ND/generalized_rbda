#pragma once

#include "config.h"
#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Dynamics/RigidBodyTreeModel.h"
#include "grbda/Utils/Utilities.h"

#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> jointMap(
    const grbda::RigidBodyTreeModel<Scalar> &grbda_model,
    const pinocchio::Model &pin_model)
{
    // TODO(@MatthewChignoli): Assumes nq = nv, and nv_i = 1 for all joints
    using EigMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    EigMat joint_map = EigMat::Zero(grbda_model.getNumDegreesOfFreedom(),
                                    grbda_model.getNumDegreesOfFreedom());
    int i = 0;
    for (const auto &name : pin_model.names)
    {
        for (const auto &node : grbda_model.rigidBodyNodes())
        {
            if (node->joint_->name_ == name)
            {
                joint_map(i, node->velocity_index_) = 1;
                i++;
            }
        }
    }
    return joint_map;
}

const std::string urdf_directory = SOURCE_DIRECTORY "/robot-models/";

struct RobotSpecification
{
    std::string urdf_filename;
    bool floating_base;
};
