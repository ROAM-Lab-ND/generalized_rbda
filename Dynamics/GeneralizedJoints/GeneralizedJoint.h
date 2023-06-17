#pragma once

#include <memory>

#include "Dynamics/Body.h"
#include "Dynamics/Joints/Joint.h"
#include "Utils/Utilities/SpatialTransforms.h"

namespace grbda
{

    using JointPtr = std::shared_ptr<Joints::Base>;

    enum class GeneralizedJointTypes
    {
        Free,
        Revolute,
        RevolutePair,
        RevolutePairWithRotor,
        RevoluteWithRotor,
        RevoluteWithMultipleRotorsJoint,
        Generic,
	TelloHipDifferential
    };

    namespace GeneralizedJoints
    {

        // TODO(@MatthewChignoli): Should probably be templated on the state types
        class Base
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Base(int num_independent_positions, int num_independent_velocities, int num_bodies);
            virtual ~Base() {}

            virtual GeneralizedJointTypes type() const = 0;

            virtual void updateKinematics(const State<double> &joint_pos,
                                          const State<double> &joint_vel) = 0;

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
            virtual int numUnactuatedVelocities() const { return 0; }

            const DMat<double> &S() const { return S_; }
            const DMat<double> &S_ring() const { return S_ring_; }
            const DMat<double> &Psi() const { return Psi_; }
            const DVec<double> &vJ() const { return vJ_; }

            // TODO(@MatthewChignoli): We should not need to static cast...
            const DVec<double> gamma(IndependentState<double> q) const
            {
                return gamma_(static_cast<DVec<double>>(q));
            }
            const DMat<double> &G() const { return G_; }
            const DVec<double> &g() const { return g_; }

            const DMat<double> &K() const { return K_; }
            const DVec<double> &k() const { return k_; }

            const DMat<double> &spanningTreeToIndependentCoordsConversion() const
            {
                return spanning_tree_to_independent_coords_conversion_;
            }

        protected:
            // TODO(@MatthewChignoli): I think we need the following function
            SpanningState<double> toSpanningTreePositions(const State<double> &joint_pos) const
            {
                return joint_pos.isSpanning() ? static_cast<DVec<double>>(joint_pos)
                                              : gamma_(joint_pos);
            }

            SpanningState<double> toSpanningTreeVelocities(const State<double> &joint_vel) const
            {
                return joint_vel.isSpanning() ? static_cast<DVec<double>>(joint_vel)
                                              : G_ * joint_vel;
            }

            const int num_bodies_;
            const int num_independent_positions_;
            const int num_independent_velocities_;

            DMat<double> S_;
            DMat<double> S_ring_;
            DMat<double> Psi_;
            DVec<double> vJ_;

            // TODO(@MatthewChignoli): This should a std::function that takes IndState as input and returns a SpanningState
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

} // namespace grbda
