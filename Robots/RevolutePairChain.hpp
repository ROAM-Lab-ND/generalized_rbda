#pragma once

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N>
    class RevolutePairChain : public SerialChain
    {
    public:
        RevolutePairChain(const bool random_parameters = true)
            : SerialChain(random_parameters)
        {
            if (N % 2 != 0)
            {
                throw std::invalid_argument("N must be even");
            }
        }

        size_t getNumDofs() const override { return N; }

    private:
        ClusterTreeModel buildRandomClusterTreeModel() const override;
        ClusterTreeModel buildUniformClusterTreeModel() const override;
    };

} // namespace grbda
