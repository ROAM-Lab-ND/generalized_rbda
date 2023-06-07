#pragma once

#include "SerialChain.hpp"

template <size_t N>
class RevolutePairChainWithRotor : public SerialChain
{
public:
    RevolutePairChainWithRotor(const bool random_parameters = true)
        : SerialChain(random_parameters)
    {
        if (N % 2 != 0)
        {
            throw std::invalid_argument("N must be even");
        }
    }

    size_t getNumDofs() const override { return N; }

    DVec<double> forwardDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const override;

    DVec<double> forwardDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const override;

    DVec<double> forwardDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const override;

private:
    ClusterTreeModel buildRandomClusterTreeModel() const override;
    ClusterTreeModel buildUniformClusterTreeModel() const override;
};
