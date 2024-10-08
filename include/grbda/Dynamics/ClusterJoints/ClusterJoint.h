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

            std::shared_ptr<LoopConstraint::Base<Scalar>> cloneLoopConstraint() const
            {
                return loop_constraint_->clone();
            }

            virtual JointState<double> randomJointState() const;

            const DMat<Scalar> &G() const { return loop_constraint_->G(); }
            const DVec<Scalar> &g() const { return loop_constraint_->g(); }

            const DMat<Scalar> &K() const { return loop_constraint_->K(); }
            const DVec<Scalar> &k() const { return loop_constraint_->k(); }

            const DMat<int> &spanningTreeToIndependentCoordsConversion() const
            {
                return spanning_tree_to_independent_coords_conversion_;
            }

            JointState<Scalar> toSpanningTreeState(const JointState<Scalar> &joint_state);

        protected:
            const int num_bodies_;
            const int num_positions_;
            const int num_velocities_;

            DMat<Scalar> S_;
            DMat<Scalar> Psi_;
            DVec<Scalar> vJ_;
            DVec<Scalar> cJ_;

            std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint_;
            std::vector<JointPtr<Scalar>> single_joints_;

            DMat<int> spanning_tree_to_independent_coords_conversion_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_H
