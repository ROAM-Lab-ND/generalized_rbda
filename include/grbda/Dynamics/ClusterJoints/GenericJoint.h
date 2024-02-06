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
            using CasadiScalar = typename std::conditional<std::is_same<Scalar, casadi::SX>::value, casadi::SX, casadi::DM>::type;
            using CasadiResult = typename std::conditional<std::is_same<Scalar, float>::value, double, Scalar>::type;

            GenericImplicit(std::vector<bool> is_coordinate_independent, SymPhiFcn phi_fcn);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<GenericImplicit<Scalar>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;

        private:
            static DMat<Scalar> runCasadiFcn(const casadi::Function &fcn,
                                             const std::vector<DVec<Scalar>> &args);

            casadi::Function K_fcn_;
            casadi::Function G_fcn_;
            casadi::Function k_fcn_;
            casadi::Function g_fcn_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Generic : public Implicit<Scalar>
        {
        public:
            // TODO(@MatthewChignoli): Maybe use pairs of bodies and joints instead of vectors? To make sure that the vectors are in the same order
            Generic(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint);

            ClusterJointTypes type() const override { return ClusterJointTypes::Generic; }

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

        private:
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body<Scalar> &getBody(const int body_index) const;

            const std::vector<Body<Scalar>> bodies_;

            DMat<Scalar> S_spanning_;
            DMat<Scalar> X_intra_;
            DMat<Scalar> X_intra_ring_;
            DMat<bool> connectivity_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
