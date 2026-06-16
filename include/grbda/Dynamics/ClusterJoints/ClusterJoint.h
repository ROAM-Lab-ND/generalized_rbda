#ifndef GRBDA_GENERALIZED_JOINT_H
#define GRBDA_GENERALIZED_JOINT_H

#include <memory>

#include "grbda/Dynamics/ClusterJoints/LoopConstraint.h"
#include "grbda/Dynamics/ClusterJoints/Transmissions.h"
#include "grbda/Dynamics/Joints/Joint.h"
#include "grbda/Utils/SpatialTransforms.h"

namespace grbda
{
    template <typename Scalar>
    using JointPtr = std::shared_ptr<Joints::Base<Scalar>>;

    enum class ClusterJointTypes
    {
        FourBar,
        Free,
        Generic,
        Revolute,
        RevolutePair,
        RevolutePairWithRotor,
        RevoluteTripleWithRotor,
        RevoluteWithRotor,
        TelloHipDifferential,
        TelloKneeAnkleDifferential
    };

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Base
        {
        public:
            Base(int num_bodies, int num_positions, int num_velocities);
            virtual ~Base() {}

            virtual ClusterJointTypes type() const = 0;

            virtual void updateKinematics(const JointState<Scalar> &joint_state) = 0;

            virtual void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const = 0;

            const std::vector<JointPtr<Scalar>> singleJoints() const { return single_joints_; };

            virtual std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const
            {
                throw std::runtime_error("Reflected Inertia not setup for this generalized joint type");
            }

            const int &numPositions() const { return num_positions_; }
            const int &numVelocities() const { return num_velocities_; }
            virtual int numUnactuatedVelocities() const { return 0; }

            const DMat<Scalar> &S() const { return S_; }
            const DMat<Scalar> &Psi() const { return Psi_; }
            const DVec<Scalar> &vJ() const { return vJ_; }
            const DVec<Scalar> &cJ() const { return cJ_; }
            const DMat<Scalar> &S_ring() const { return S_ring_; }

            // Derivative interface for configuration-dependent motion subspaces
            // Returns zero by default for cluster joints
            // Override for joints with absolute coordinates or configuration-dependent kinematics

            // Returns ∂(Ṡ·q̇)/∂q as a (6*num_bodies x nv) matrix
            virtual void getSdotqd_q(DMat<Scalar>& out) const {
                out.setZero(num_bodies_ * 6, num_velocities_);
            }

            // Contraction-based derivative interface
            // Default returns zero (for joints with constant S). Override for configuration-dependent S.

            // Returns true if this joint has configuration-dependent motion subspace S(q)
            virtual bool hasConfigurationDependentS() const { return false; }

            // Returns ∂(S*b)/∂q as a (6*num_bodies x nv) matrix
            virtual void evalSTimesVec_dq(const DVec<Scalar>& b, DMat<Scalar>& out) const {
                (void)b;
                out.resize(0, 0);
            }

            // Returns ∂(S^T*F)/∂q as a (nv x nv) matrix
            virtual void evalSTTimesVec_dq(const DVec<Scalar>& F, DMat<Scalar>& out) const {
                (void)F;
                out.resize(0, 0);
            }


            std::shared_ptr<LoopConstraint::Base<Scalar>> cloneLoopConstraint() const
            {
                return loop_constraint_->clone();
            }

            virtual JointState<double> randomJointState(bool enforce_position_constraint = true) const;

            const DMat<Scalar> &G() const { return loop_constraint_->G(); }
            const DVec<Scalar> &g() const { return loop_constraint_->g(); }

            const DMat<Scalar> &K() const { return loop_constraint_->K(); }
            const DVec<Scalar> &k() const { return loop_constraint_->k(); }

            bool isImplicit() const { return loop_constraint_->isImplicit(); }
            DVec<Scalar> phi(const JointCoordinate<Scalar> &joint_pos) const
            {
                return loop_constraint_->phi(joint_pos);
            }
            void updateJacobians(const JointCoordinate<Scalar> &joint_pos)
            {
                loop_constraint_->updateJacobians(joint_pos);
            }

            const DMat<int> &spanningTreeToIndependentCoordsConversion() const
            {
                return spanning_tree_to_independent_coords_conversion_;
            }

            JointState<Scalar> toSpanningTreeState(const JointState<Scalar> &joint_state,
                                                   bool enforce_constraints = false);

        protected:
            const int num_bodies_;
            const int num_positions_;
            const int num_velocities_;

            DMat<Scalar> S_;
            DMat<Scalar> Psi_;
            DVec<Scalar> vJ_;
            DVec<Scalar> cJ_;
            DMat<Scalar> S_ring_;

            std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint_;
            std::vector<JointPtr<Scalar>> single_joints_;

            DMat<int> spanning_tree_to_independent_coords_conversion_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_H
