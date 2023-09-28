#ifndef GRBDA_ROBOTS_REVOLUTE_TRIPLE_CHAIN_WITH_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_TRIPLE_CHAIN_WITH_ROTOR_H

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N>
    class RevoluteTripleChainWithRotor : public SerialChain<double>
    {
    public:
        RevoluteTripleChainWithRotor(const bool random_parameters = true)
            : SerialChain(random_parameters)
        {
            if (N % 3 != 0)
            {
                throw std::invalid_argument("N must be divisible by 3");
            }
        }

        size_t getNumDofs() const override { return N; }

    private:
        ClusterTreeModel<> buildRandomClusterTreeModel() const override;
        ClusterTreeModel<> buildUniformClusterTreeModel() const override;

        void appendContactPoints(ClusterTreeModel<> &model, const int i,
                                 const std::string linkA_name,
                                 const std::string linkB_name,
                                 const std::string linkC_name) const;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_TRIPLE_CHAIN_WITH_ROTOR_H
