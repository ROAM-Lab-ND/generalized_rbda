#pragma once

#include <memory>

#include "LoopConstraint.h"
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
        TelloHipDifferential,
        TelloKneeAnkleDifferential
    };

    namespace GeneralizedJoints
    {

        class Base
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Base(int num_bodies, int num_independent_positions, int num_independent_velocities,
                 bool position_is_spanning, bool velocity_is_spanning);
            virtual ~Base() {}

            virtual GeneralizedJointTypes type() const = 0;

            virtual void updateKinematics(const JointState &joint_state) = 0;

            virtual void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const = 0;

            const std::vector<JointPtr> singleJoints() const { return single_joints_; };

            virtual std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const
            {
                throw std::runtime_error("Reflected Inertia not setup for this generalized joint type");
            }

            const int &numPositions() const { return num_positions_; }
            const int &numVelocities() const { return num_velocities_; }
            virtual int numUnactuatedVelocities() const { return 0; }

            const bool &positionIsSpanning() const { return position_is_spanning_; }
            const bool &velocityIsSpanning() const { return velocity_is_spanning_; }

            const DMat<double> &S() const { return S_; }
            const DMat<double> &S_ring() const { return S_ring_; }
            const DMat<double> &Psi() const { return Psi_; }
            const DVec<double> &vJ() const { return vJ_; }

            std::shared_ptr<LoopConstraint::Base> cloneLoopConstraint() const
            {
                return loop_constraint_->clone();
            }

            virtual JointState randomJointState() const;

            const DMat<double> &G() const { return loop_constraint_->G(); }
            const DVec<double> &g() const { return loop_constraint_->g(); }

            const DMat<double> &K() const { return loop_constraint_->K(); }
            const DVec<double> &k() const { return loop_constraint_->k(); }

            const DMat<double> &spanningTreeToIndependentCoordsConversion() const
            {
                return spanning_tree_to_independent_coords_conversion_;
            }

            JointState toSpanningTreeState(const JointState &joint_state);

        protected:
#ifdef DEBUG_MODE
            void jointStateCheck(const JointState &joint_state) const
            {
                if (joint_state.position.isSpanning() != position_is_spanning_)
                    throw std::runtime_error("Position is in the wrong coordinates");
                if (joint_state.velocity.isSpanning() != velocity_is_spanning_)
                    throw std::runtime_error("Velocity is in the wrong coordinates");
                if (joint_state.position.rows() != num_positions_)
                    throw std::runtime_error("Position is the wrong size");
                if (joint_state.velocity.rows() != num_velocities_)
                    throw std::runtime_error("Velocity is the wrong size");
            }
#endif

            const int num_bodies_;
            const int num_positions_;
            const int num_velocities_;

            const bool position_is_spanning_;
            const bool velocity_is_spanning_;

            DMat<double> S_;
            DMat<double> S_ring_;
            DMat<double> Psi_;
            DVec<double> vJ_;

            std::shared_ptr<LoopConstraint::Base> loop_constraint_;
            std::vector<JointPtr> single_joints_;

            // TODO(@MatthewChignoli): Eventually delete all of this stuff
            DMat<double> spanning_tree_to_independent_coords_conversion_;
        };

    }

} // namespace grbda
