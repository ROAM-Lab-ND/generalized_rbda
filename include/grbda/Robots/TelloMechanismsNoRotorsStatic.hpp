#ifndef GRBDA_ROBOTS_TELLO_MECHANISMS_NO_ROTORS_STATIC_H
#define GRBDA_ROBOTS_TELLO_MECHANISMS_NO_ROTORS_STATIC_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{
    /**
     * @brief Tello robot variant with static (linear) constraint mechanisms but without rotors.
     *
     * This class creates a Tello model that:
     * - Has 4-coordinate clusters (matching full Tello structure)
     * - Uses STATIC constraint matrices (constant gear ratios, no CasADi)
     * - Removes all rotor bodies (replaced with negligible virtual rotors)
     *
     * This allows isolating the computational overhead of constraint structure
     * separate from CasADi symbolic differentiation.
     *
     * Constraint approximation: Instead of complex four-bar linkage equations,
     * we use static linear relationships:
     *   q_gimbal ≈ gear_ratio * q_rotor1
     *   q_thigh  ≈ gear_ratio * q_rotor2
     */
    template <typename Scalar>
    class TelloMechanismsNoRotorsStatic : public Tello<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TelloMechanismsNoRotorsStatic() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_MECHANISMS_NO_ROTORS_STATIC_H
