#pragma once

#include <memory>

#include "DynamicsEngine/Body.h"
#include "DynamicsEngine/Joints/Joint.h"
#include "Utils/Utilities/SpatialTransforms.h"

using JointPtr = std::shared_ptr<Joints::Base>;

enum class GeneralizedJointTypes
{
    Free,
    Revolute,
    RevolutePair,
    RevolutePairWithRotor,
    RevoluteWithRotor,
    RevoluteWithMultipleRotorsJoint,
    Generic
};

namespace GeneralizedJoints
{

class Base
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Base(int num_independent_positions, int num_independent_velocities, int num_bodies);
    virtual ~Base() {}

    virtual GeneralizedJointTypes type() const = 0;

    virtual void updateKinematics(const DVec<double> &y, const DVec<double> &yd) = 0;
    virtual void computeSpatialTransformFromParentToCurrentCluster(
        GeneralizedSpatialTransform &Xup) const = 0;

    const std::vector<JointPtr> singleJoints() const { return single_joints_; };    

    virtual std::vector<std::tuple<Body, JointPtr, DMat<double>>>
    bodiesJointsAndReflectedInertias() const
    {
        throw std::runtime_error("Reflected Inertia not setup for this generalized joint type");
    }

    int numPositions() const { return num_independent_positions_; }
    int numVelocities() const { return num_independent_velocities_; }

    const DMat<double> &S() const { return S_; }
    const DMat<double> &S_ring() const { return S_ring_; }
    const DMat<double> &Psi() const { return Psi_; }
    const DVec<double> &vJ() const { return vJ_; }

    const DVec<double> gamma(DVec<double> q) const { return gamma_(q); }
    const DMat<double> &G() const { return G_; }
    const DVec<double> &g() const { return g_; }

    const DMat<double> &K() const { return K_; }
    const DVec<double> &k() const { return k_; }
    
    const DMat<double> &spanningTreeToIndependentCoordsConversion() const
    {
        return spanning_tree_to_independent_coords_conversion_;
    }

protected:
    const int num_bodies_;
    const int num_independent_positions_;
    const int num_independent_velocities_;

    DMat<double> S_;
    DMat<double> S_ring_;
    DMat<double> Psi_;
    DVec<double> vJ_;

    DVecFcn<double> gamma_;
    DMat<double> G_;
    DVec<double> g_;

    DVecFcn<double> phi_;
    DMat<double> K_;
    DVec<double> k_;

    DMat<double> spanning_tree_to_independent_coords_conversion_;

    std::vector<JointPtr> single_joints_;
};

}
