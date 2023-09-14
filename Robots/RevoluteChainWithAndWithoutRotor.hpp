#ifndef GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_AND_WITHOUT_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_AND_WITHOUT_ROTOR_H

#include "SerialChain.hpp"

namespace grbda
{

    // TODO(@MatthewChignoli): Maybe need better names for N and M
    template <size_t N, size_t M>
    class RevoluteChainWithAndWithoutRotor : public SerialChain
    {
    public:
        RevoluteChainWithAndWithoutRotor(bool random_parameters = true)
            : SerialChain(random_parameters) {}

        size_t getNumDofs() const override { return N + M; }

    private:
        ClusterTreeModel buildRandomClusterTreeModel() const override;
        ClusterTreeModel buildUniformClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_AND_WITHOUT_ROTOR_H