#ifndef GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_H
#define GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_H

#include "grbda/Robots/SerialChains/SerialChain.hpp"

namespace grbda
{

    template <size_t N, typename Scalar = double>
    class RevolutePairChain : public SerialChain<Scalar>
    {
    public:
        typedef typename ClusterJoints::RevolutePair<Scalar> RevolutePair;
        
        RevolutePairChain(const bool random_parameters = true)
            : SerialChain<Scalar>(random_parameters)
        {
            if (N % 2 != 0)
            {
                throw std::invalid_argument("N must be even");
            }
        }

        size_t getNumDofs() const override { return N; }

    private:
        ClusterTreeModel<Scalar> buildRandomClusterTreeModel() const override;
        ClusterTreeModel<Scalar> buildUniformClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_H
