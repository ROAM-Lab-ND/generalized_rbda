#ifndef GRBDA_PINOCCHIO_HELPERS_H
#define GRBDA_PINOCCHIO_HELPERS_H

#include <iostream>
#include <fstream>

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

#endif // GRBDA_PINOCCHIO_HELPERS_H
