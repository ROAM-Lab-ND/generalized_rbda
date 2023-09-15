#ifndef GRBDA_GENERALIZED_JOINT_H
#define GRBDA_GENERALIZED_JOINT_H

#include <memory>

#include "LoopConstraint.h"
#include "Dynamics/Body.h"
#include "Dynamics/Joints/Joint.h"
#include "Utils/SpatialTransforms.h"

namespace grbda
{

    using JointPtr = std::shared_ptr<Joints::Base>;

    enum class GeneralizedJointTypes
    {
        Free,
        Revolute,
        RevolutePair,
        RevolutePairWithRotor,
        RevoluteTripleWithRotor,
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

            Base(int num_bodies, int num_independent_positions, int num_independent_velocities);
            virtual ~Base() {}

            virtual GeneralizedJointTypes type() const = 0;

            virtual void updateKinematics(const JointState &joint_state) = 0;

            virtual void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform &Xup) const = 0;

            const std::vector<JointPtr> singleJoints() const { return single_joints_; };

            virtual std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const
            {
                throw std::runtime_error("Reflected Inertia not setup for this generalized joint type");
            }

            const int &numPositions() const { return num_positions_; }
            const int &numVelocities() const { return num_velocities_; }
            virtual int numUnactuatedVelocities() const { return 0; }

            virtual JointCoordinate integratePosition(JointState joint_state, double dt) const;

            const DMat<double> &S() const { return S_; }
            const DMat<double> &Psi() const { return Psi_; }
            const DVec<double> &vJ() const { return vJ_; }
            const DVec<double> &cJ() const { return cJ_; }

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
            const int num_bodies_;
            const int num_positions_;
            const int num_velocities_;

            DMat<double> S_;
            DMat<double> Psi_;
            DVec<double> vJ_;
            DVec<double> cJ_;

            std::shared_ptr<LoopConstraint::Base> loop_constraint_;
            std::vector<JointPtr> single_joints_;

            DMat<double> spanning_tree_to_independent_coords_conversion_;
        };

        struct GearedTransmissionModule
        {
            Body body_;
            Body rotor_;
            ori::CoordinateAxis joint_axis_;
            ori::CoordinateAxis rotor_axis_;
            double gear_ratio_;
        };

        struct ParallelBeltTransmissionModule
        {
            Body body_;
            Body rotor_;
            ori::CoordinateAxis joint_axis_;
            ori::CoordinateAxis rotor_axis_;
            double gear_ratio_;
            double belt_ratio_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_H
