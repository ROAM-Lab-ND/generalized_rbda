#ifndef GRBDA_ROBOTS_TELLO_NO_ROTORS_H
#define GRBDA_ROBOTS_TELLO_NO_ROTORS_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{

    // Tello robot without rotors or four-bar linkage mechanisms.
    // Uses plain Revolute joints for all leg DOFs.
    // This provides an apples-to-apples comparison with Tello and TelloNoMechanisms
    // at the same 16 DOF (6 floating base + 10 leg joints).
    template <typename Scalar>
    class TelloNoRotors : public Tello<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TelloNoRotors() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_NO_ROTORS_H
