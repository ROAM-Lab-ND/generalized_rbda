#ifndef GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_AND_WITHOUT_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_AND_WITHOUT_ROTOR_H

#include "grbda/Robots/SerialChains/SerialChain.hpp"

namespace grbda
{

    template <size_t N, size_t M, typename Scalar = double>
    class RevoluteChainWithAndWithoutRotor : public SerialChain<Scalar>
    {
    public:
        typedef typename ClusterJoints::RevoluteWithRotor<Scalar> RevoluteWithRotor;
        typedef typename ClusterJoints::Revolute<Scalar> Revolute;
        typedef typename ClusterJoints::GearedTransmissionModule<Scalar> TransmissionModule;

        RevoluteChainWithAndWithoutRotor(bool random_parameters = true)
            : SerialChain<Scalar>(random_parameters) {}

        size_t getNumDofs() const override { return N + M; }

    private:
        ClusterTreeModel<Scalar> buildRandomClusterTreeModel() const override;
        ClusterTreeModel<Scalar> buildUniformClusterTreeModel() const override;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_AND_WITHOUT_ROTOR_H
