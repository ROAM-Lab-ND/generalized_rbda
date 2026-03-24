#ifndef GRBDA_ROBOTS_TELLO_MECHANISMS_NO_ROTORS_H
#define GRBDA_ROBOTS_TELLO_MECHANISMS_NO_ROTORS_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{
    /**
     * @brief Tello robot variant with mechanisms but without rotors.
     *
     * This class creates a Tello model that:
     * - Keeps the four-bar linkage mechanisms (Generic clusters with implicit constraints)
     * - Removes all rotor bodies
     *
     * This allows isolating the computational overhead of the mechanisms
     * separate from the rotor overhead.
     */
    template <typename Scalar>
    class TelloMechanismsNoRotors : public Tello<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TelloMechanismsNoRotors() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_MECHANISMS_NO_ROTORS_H
