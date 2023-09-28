#ifndef GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_H
#define GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_H

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N>
    class RevolutePairChain : public SerialChain<double>
    {
    public:
        RevolutePairChain(const bool random_parameters = true) : SerialChain(random_parameters)
        {
            if (N % 2 != 0)
            {
                throw std::invalid_argument("N must be even");
            }
        }

        size_t getNumDofs() const override { return N; }

    private:
        ClusterTreeModel<> buildRandomClusterTreeModel() const override;
        ClusterTreeModel<> buildUniformClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_H
