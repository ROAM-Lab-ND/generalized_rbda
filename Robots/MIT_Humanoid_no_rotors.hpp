#ifndef GRBDA_ROBOTS_MIT_HUMANOID_NO_ROTOR_H
#define GRBDA_ROBOTS_MIT_HUMANOID_NO_ROTOR_H

#include "MIT_Humanoid.hpp"

namespace grbda
{

    template <typename Scalar, typename OrientationRepresentation = ori_representation::QuaternionRepresentation<Scalar>>
    class MIT_Humanoid_no_rotors : public MIT_Humanoid<Scalar, OrientationRepresentation>
    {
    public:
        MIT_Humanoid_no_rotors() {}
        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_MIT_HUMANOID_NO_ROTOR_H
