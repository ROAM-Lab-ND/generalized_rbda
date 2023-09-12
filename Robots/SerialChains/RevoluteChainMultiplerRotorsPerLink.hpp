#ifndef GRBDA_ROBOTS_REVOLUTE_CHAIN_MULTIPLE_ROTORS_PER_LINK_H
#define GRBDA_ROBOTS_REVOLUTE_CHAIN_MULTIPLE_ROTORS_PER_LINK_H

#include "SerialChain.hpp"

namespace grbda
{

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

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_CHAIN_MULTIPLE_ROTORS_PER_LINK_H
