#ifndef GRBDA_ROBOTS_TELLO_NO_MECHANISMS_H
#define GRBDA_ROBOTS_TELLO_NO_MECHANISMS_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{

    // Tello robot without four-bar linkage mechanisms.
    // Uses RevoluteWithRotor joints instead of Generic clusters with implicit constraints.
    // This preserves rotor dynamics but removes the differential transmission kinematics.
    template <typename Scalar>
    class TelloNoMechanisms : public Tello<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TelloNoMechanisms() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_NO_MECHANISMS_H
