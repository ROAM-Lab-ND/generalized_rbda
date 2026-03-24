#ifndef GRBDA_ROBOTS_TELLO_ROTORS_NO_CONSTRAINTS_H
#define GRBDA_ROBOTS_TELLO_ROTORS_NO_CONSTRAINTS_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{

    /// Tello robot with real rotor inertias but NO constraint couplings.
    /// Each joint (hip-clamp, gimbal, thigh, shin, foot) is an independent
    /// RevoluteWithRotor cluster with geared transmissions.
    /// 
    /// Purpose: Isolates the computational overhead of rotor inertia dynamics
    /// from the overhead of implicit loop constraints (differentials).
    /// This is the second factor in the factorial design:
    ///   - With Rotors, Without Constraints: pure rotor inertia cost
    /// 
    /// Differs from TelloNoMechanisms in that TelloNoMechanisms still applies
    /// GenericImplicit constraints for hip/knee differentials; this variant
    /// removes all constraints to get independent cluster structure.
    template <typename Scalar>
    class TelloRotorsNoConstraints : public Tello<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TelloRotorsNoConstraints() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_ROTORS_NO_CONSTRAINTS_H
