#ifndef GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_WITH_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_WITH_ROTOR_H

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N, typename Scalar = double>
    class RevolutePairChainWithRotor : public SerialChain<Scalar>
    {
    public:
        typedef typename ClusterJoints::RevolutePairWithRotor<Scalar> RevPairRotor;
        typedef typename ClusterJoints::ParallelBeltTransmissionModule<Scalar> TransmissionModule;

        RevolutePairChainWithRotor(const bool random_parameters = true)
            : SerialChain<Scalar>(random_parameters)
        {
            if (N % 2 != 0)
            {
                throw std::invalid_argument("N must be even");
            }
        }

        size_t getNumDofs() const override { return N; }

        DVec<double> forwardDynamics(
            const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const override;

        DVec<double> forwardDynamicsReflectedInertia(
            const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const override;

        DVec<double> forwardDynamicsReflectedInertiaDiag(
            const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const override;

        DVec<double> inverseDynamics(
            const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const override;

        DVec<double> inverseDynamicsReflectedInertia(
            const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const override;

        DVec<double> inverseDynamicsReflectedInertiaDiag(
            const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const override;

    private:
        ClusterTreeModel<Scalar> buildRandomClusterTreeModel() const override;
        ClusterTreeModel<Scalar> buildUniformClusterTreeModel() const override;

        void appendContactPoints(ClusterTreeModel<Scalar> &model, const int i,
                                 const std::string linkA_name, const std::string linkB_name) const;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_PAIR_CHAIN_WITH_ROTOR_H
