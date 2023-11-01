#ifndef GRBDA_ROBOTS_MIT_HUMANOID_NO_ROTOR_H
#define GRBDA_ROBOTS_MIT_HUMANOID_NO_ROTOR_H

#include "grbda/Robots/MIT_Humanoid.hpp"

namespace grbda
{

    template <typename Scalar = double,
              typename OrientationRepresentation = ori_representation::Quaternion>
    class MIT_Humanoid_no_rotors : public MIT_Humanoid<Scalar, OrientationRepresentation>
    {
    public:
        MIT_Humanoid_no_rotors() {}
        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_MIT_HUMANOID_NO_ROTOR_H
