#pragma once

#include "SerialChain.hpp"

template <size_t N, size_t M>
class RevoluteChainMultipleRotorsPerLink : public SerialChain
{
public:
    RevoluteChainMultipleRotorsPerLink(bool random_parameters = true)
        : SerialChain(random_parameters) {}

    size_t getNumDofs() const override { return N; }

private:
    ClusterTreeModel buildRandomClusterTreeModel() const override;
    ClusterTreeModel buildUniformClusterTreeModel() const override;
};
