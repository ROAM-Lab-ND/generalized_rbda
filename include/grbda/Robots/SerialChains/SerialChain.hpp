#ifndef GRBDA_ROBOTS_SERIAL_CHAIN_H
#define GRBDA_ROBOTS_SERIAL_CHAIN_H

#include "grbda/Robots/Robot.h"

namespace grbda
{

    template <typename Scalar>
    class SerialChain : public Robot<Scalar>
    {
    public:
        SerialChain(bool random_parameters = true) : _random_parameters(random_parameters) {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override
        {
            if (_random_parameters)
                return buildRandomClusterTreeModel();
            else
                return buildUniformClusterTreeModel();
        }

        virtual size_t getNumDofs() const = 0;

    protected:
        virtual ClusterTreeModel<Scalar> buildRandomClusterTreeModel() const = 0;
        virtual ClusterTreeModel<Scalar> buildUniformClusterTreeModel() const = 0;

        SpatialInertia<Scalar> randomLinkSpatialInertia() const
        {
            return SpatialInertia<Scalar>::createRandomInertia();
        }

        SpatialInertia<Scalar> randomRotorSpatialInertia() const
        {
            return SpatialInertia<Scalar>::createRandomInertia(1e-4);
        }

        Scalar randomGearRatio() const
        {
            return static_cast<Scalar>(rand() % _gear_ratio_scale + 1);
        }

        template <size_t N>
        Eigen::Matrix<Scalar, N, 1> randomBeltRatios() const
        {
            Eigen::Matrix<Scalar, N, 1> ratios;
            for (size_t i = 0; i < N; ++i)
            {
                ratios[i] = static_cast<Scalar>(rand() % _belt_ratio_scale + 1);
            }
            return ratios;
        }

        const bool _random_parameters;
        int _gear_ratio_scale = 5;
        int _belt_ratio_scale = 5;
        };

} // namespace grbda

#endif // GRBDA_ROBOTS_SERIAL_CHAIN_H
