#pragma once

#include "Robot.h"

namespace grbda
{

    class SerialChain : public Robot
    {
    public:
        SerialChain(bool random_parameters = true)
            : _random_parameters(random_parameters) {}

        ClusterTreeModel buildClusterTreeModel() const override
        {
            if (_random_parameters)
                return buildRandomClusterTreeModel();
            else
                return buildUniformClusterTreeModel();
        }

        virtual size_t getNumDofs() const = 0;

    protected:
        virtual ClusterTreeModel buildRandomClusterTreeModel() const = 0;
        virtual ClusterTreeModel buildUniformClusterTreeModel() const = 0;

        SpatialInertia<double> randomLinkSpatialInertia() const
        {
            return SpatialInertia<double>::createRandomInertia();
        }

        SpatialInertia<double> randomRotorSpatialInertia() const
        {
            return SpatialInertia<double>::createRandomInertia(1e-4);
        }

        const bool _random_parameters;
        int _gear_ratio_scale = 5;
        };

} // namespace grbda
