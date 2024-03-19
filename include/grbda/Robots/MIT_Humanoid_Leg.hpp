#ifndef GRBDA_ROBOTS_MIT_HUMANOID_LEG_H
#define GRBDA_ROBOTS_MIT_HUMANOID_LEG_H

#include "grbda/Robots/MIT_Humanoid.hpp"

namespace grbda
{

    template <typename Scalar = double>
    class MIT_Humanoid_Leg : public MIT_Humanoid<Scalar>
    {
    public:
        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_MIT_HUMANOID_LEG_H
