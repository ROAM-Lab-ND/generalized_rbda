#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    // TODO(@MatthewChignoli): Add a unit test that compares generic joint versions to specialized versions. For example, compare revolute pair with rotor as a generic joint to the specialized joint class.
    struct ExplicitConstraint
    {
        ExplicitConstraint(DVecFcn<double> gamma, DMat<double> G, DVec<double> g)
            : gamma_(gamma), G_(G), g_(g) {}

        int numIndependentVelocities() const { return G_.cols(); }

        DVecFcn<double> gamma_;
        DMat<double> G_;
        DVec<double> g_;
    };

    struct ImplicitConstraint
    {
        DVecFcn<double> phi_;
        DMat<double> K_;
        DVec<double> k_;
    };

    namespace GeneralizedJoints
    {

        class Generic : public Base
        {
        public:
            Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                    const ExplicitConstraint &explicit_constraint);
            Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                    const ImplicitConstraint &implicit_constraint);

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::Generic; }

            void updateKinematics(const JointState &joint_state) override;
                                  
            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

        private:
            void extractImplicitConstraintFromExplicit(const ExplicitConstraint &explicit_constraint);
            void extractExplicitConstraintFromImplicit(const ImplicitConstraint &implicit_constraint) {}
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body &getBody(const int body_index) const;

            const std::vector<Body> bodies_;

            DMat<double> S_spanning_tree_;
            DMat<double> Xup_spanning_tree_;
            DMat<double> Xup_ring_spanning_tree_;
            DMat<bool> connectivity_;
        };

    }

} // namespace grbda
