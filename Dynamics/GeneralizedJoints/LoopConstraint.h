#pragma once

// #include <memory>
// #include "Dynamics/Body.h"
// #include "Dynamics/Joints/Joint.h"
// #include "Utils/Utilities/SpatialTransforms.h"
#include "Utils/cppTypes.h"

// TODO(@MatthewChignoli): The constraints are the abstract class. Then we can have stuff inherit from them.

// TODO(@MatthewChignoli): And then we need an explicit constraint collection class (just a std::Vector)

// TODO(@MatthewChignoli): Should probably make a namespace...

namespace grbda
{

    namespace LoopConstraint
    {

        struct Base
        {
            virtual ~Base() {}

            virtual void updateJacobians(const JointCoordinate &joint_pos) = 0;
            virtual void updateBiases(const JointState &joint_state) = 0;

            virtual DVec<double> gamma(const JointCoordinate &joint_pos) const = 0;
            DMat<double> G() const { return G_;}
            DVec<double> g() const { return g_;}

            DMat<double> K() const { return K_;}
            DVec<double> k() const { return k_;}

        protected:
            DMat<double> G_;
            DVec<double> g_;

            DMat<double> K_;
            DVec<double> k_;
        };

        struct Static : Base
        {
            Static(DMat<double> G, DMat<double> K)
            {
                G_ = G;
                g_ = DVec<double>::Zero(G.rows());

                K_ = K;
                k_ = DVec<double>::Zero(K.rows());
            }

            void updateJacobians(const JointCoordinate &joint_pos) override {}
            void updateBiases(const JointState &joint_state) override {}

            DVec<double> gamma(const JointCoordinate &joint_pos) const override
            {
                return G_ * joint_pos;
            }
        };

    }

}
