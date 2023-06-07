#pragma once

#include "SerialChain.hpp"

template <size_t N>
class RevoluteChainWithRotor : public SerialChain
{
public:
    // TODO(@MatthewChignoli): I don't like passing this bool, I think a better option is to make a child class called "RandomRevoluteChainWithRotor"
    RevoluteChainWithRotor(bool random_parameters = true)
        : SerialChain(random_parameters) {}

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
