#ifndef GRBDA_GENERALIZED_JOINT_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_GENERIC_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar = double>
        struct GenericImplicit : Base<Scalar>
        {
            using SX = casadi::SX;
            using SymPhiFcn = std::function<DVec<SX>(const JointCoordinate<SX> &)>;

            GenericImplicit(std::vector<bool> is_coordinate_independent, SymPhiFcn phi_fcn);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<GenericImplicit<Scalar>>(*this);
            }

            GenericImplicit<double> copyAsDouble() const
            {
                return GenericImplicit<double>(this->is_coordinate_independent_, phi_sym_);
            }

            GenericImplicit<SX> copyAsSymbolic() const
            {
                return GenericImplicit<SX>(this->is_coordinate_independent_, phi_sym_);
            }

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override;
            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;
            
            const std::vector<bool>& isCoordinateIndependent() const;

            void createRandomStateHelpers() override;

        private:
            static DMat<Scalar> runCasadiFcn(const casadi::Function &fcn,
                                             const JointCoordinate<Scalar> &arg);
            static DMat<Scalar> runCasadiFcn(const casadi::Function &fcn,
                                             const JointState<Scalar> &args);

            const std::vector<bool> is_coordinate_independent_;
            SymPhiFcn phi_sym_;

            casadi::Function K_fcn_;
            casadi::Function G_fcn_;
            casadi::Function k_fcn_;
            casadi::Function g_fcn_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Generic : public Base<Scalar>
        {
        public:
            Generic(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint);

            Generic(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> loop_constraint);

            ClusterJointTypes type() const override { return ClusterJointTypes::Generic; }

            JointState<double> randomJointState() const override;

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

        private:
            void initialize(const std::vector<JointPtr<Scalar>> &joints,
                            std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint);

            JointCoordinate<double> findRootsForPhi() const;
        
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body<Scalar> &getBody(const int body_index) const;

            const std::vector<Body<Scalar>> bodies_;
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> generic_constraint_;

            DMat<Scalar> S_spanning_;
            DMat<Scalar> X_intra_;
            DMat<Scalar> X_intra_ring_;
            DMat<bool> connectivity_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
