#ifndef GRBDA_ROBOTS_REVOLUTE_TRIPLE_CHAIN_WITH_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_TRIPLE_CHAIN_WITH_ROTOR_H

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N, typename Scalar = double>
    class RevoluteTripleChainWithRotor : public SerialChain<Scalar>
    {
    public:
        typedef typename ClusterJoints::RevoluteTripleWithRotor<Scalar> RevTripleWithRotor;
        typedef typename ClusterJoints::ParallelBeltTransmissionModule<Scalar> TransmissionModule;

        RevoluteTripleChainWithRotor(const bool random_parameters = true)
            : SerialChain<Scalar>(random_parameters)
        {
            if (N % 3 != 0)
            {
                throw std::invalid_argument("N must be divisible by 3");
            }
        }

        size_t getNumDofs() const override { return N; }

    private:
        ClusterTreeModel<Scalar> buildRandomClusterTreeModel() const override;
        ClusterTreeModel<Scalar> buildUniformClusterTreeModel() const override;

        void appendContactPoints(ClusterTreeModel<Scalar> &model, const int i,
                                 const std::string linkA_name,
                                 const std::string linkB_name,
                                 const std::string linkC_name) const;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_TRIPLE_CHAIN_WITH_ROTOR_H
