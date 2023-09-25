#ifndef GRBDA_GENERALIZED_JOINT_H
#define GRBDA_GENERALIZED_JOINT_H

#include <memory>

#include "LoopConstraint.h"
#include "Dynamics/Body.h"
#include "Dynamics/Joints/Joint.h"
#include "Utils/SpatialTransforms.h"

namespace grbda
{
    template <typename Scalar>
    using JointPtr = std::shared_ptr<Joints::Base<Scalar>>;

    enum class ClusterJointTypes
    {
        Free,
        Revolute,
        RevolutePair,
        RevolutePairWithRotor,
        RevoluteTripleWithRotor,
        RevoluteWithRotor,
        Generic,
        TelloHipDifferential,
        TelloKneeAnkleDifferential
    };

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Base
        {
        public:
            Base(int num_bodies, int num_independent_positions, int num_independent_velocities);
            virtual ~Base() {}

            virtual ClusterJointTypes type() const = 0;

            virtual void updateKinematics(const JointState<> &joint_state) = 0;

            virtual void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<> &Xup) const = 0;

            const std::vector<JointPtr<double>> singleJoints() const { return single_joints_; };

            virtual std::vector<std::tuple<Body<>, JointPtr<double>, DMat<double>>>
            bodiesJointsAndReflectedInertias() const
            {
                throw std::runtime_error("Reflected Inertia not setup for this generalized joint type");
            }

            const int &numPositions() const { return num_positions_; }
            const int &numVelocities() const { return num_velocities_; }
            virtual int numUnactuatedVelocities() const { return 0; }

            virtual JointCoordinate<> integratePosition(JointState<> joint_state, double dt) const;

            const DMat<double> &S() const { return S_; }
            const DMat<double> &Psi() const { return Psi_; }
            const DVec<double> &vJ() const { return vJ_; }
            const DVec<double> &cJ() const { return cJ_; }

            std::shared_ptr<LoopConstraint::Base<Scalar>> cloneLoopConstraint() const
            {
                return loop_constraint_->clone();
            }

            virtual JointState<> randomJointState() const;

            const DMat<double> &G() const { return loop_constraint_->G(); }
            const DVec<double> &g() const { return loop_constraint_->g(); }

            const DMat<double> &K() const { return loop_constraint_->K(); }
            const DVec<double> &k() const { return loop_constraint_->k(); }

            const DMat<double> &spanningTreeToIndependentCoordsConversion() const
            {
                return spanning_tree_to_independent_coords_conversion_;
            }

            JointState<> toSpanningTreeState(const JointState<> &joint_state);

        protected:
            const int num_bodies_;
            const int num_positions_;
            const int num_velocities_;

            DMat<double> S_;
            DMat<double> Psi_;
            DVec<double> vJ_;
            DVec<double> cJ_;

            std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint_;
            std::vector<JointPtr<double>> single_joints_;

            DMat<double> spanning_tree_to_independent_coords_conversion_;
        };

        template <typename Scalar = double>
        struct GearedTransmissionModule
        {
            Body<Scalar> body_;
            Body<Scalar> rotor_;
            ori::CoordinateAxis joint_axis_;
            ori::CoordinateAxis rotor_axis_;
            Scalar gear_ratio_;
        };

        template <typename Scalar = double>
        struct ParallelBeltTransmissionModule
        {
            Body<Scalar> body_;
            Body<Scalar> rotor_;
            ori::CoordinateAxis joint_axis_;
            ori::CoordinateAxis rotor_axis_;
            Scalar gear_ratio_;
            Scalar belt_ratio_;
        };

        template <typename Scalar = double>
        struct TelloDifferentialModule
        {
            Body<Scalar> rotor1_;
            Body<Scalar> rotor2_;
            Body<Scalar> link1_;
            Body<Scalar> link2_;
            ori::CoordinateAxis rotor1_axis_;
            ori::CoordinateAxis rotor2_axis_;
            ori::CoordinateAxis link1_axis_;
            ori::CoordinateAxis link2_axis_;
            Scalar gear_ratio_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_H
