#ifndef GRBDA_ROBOTS_TELLO_CLUSTERED_NO_CONSTRAINTS_H
#define GRBDA_ROBOTS_TELLO_CLUSTERED_NO_CONSTRAINTS_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{
    /**
     * @brief Tello robot with real rotors in clustered structure but WITHOUT loop constraints
     *
     * This variant is designed to isolate the overhead of clustering structure alone,
     * separate from constraint solving complexity.
     *
     * Structure:
     * - Hip clamp: RevoluteWithRotor cluster (real rotor bodies, 2 DOF)
     * - Hip differential: 4-DOF cluster (2 rotor + 2 link DOFs, but NO constraints)
     * - Knee-ankle differential: 4-DOF cluster (2 rotor + 2 link DOFs, but NO constraints)
     *
     * Key difference from Tello: No loop constraints, so all 4 DOFs in each differential
     * cluster are independent (no dependency relationships).
     */
    template <typename Scalar = double>
    class TelloClusteredNoConstraints : public Tello<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TelloClusteredNoConstraints() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif
