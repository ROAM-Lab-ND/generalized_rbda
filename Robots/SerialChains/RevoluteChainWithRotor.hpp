#ifndef GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_ROTOR_H

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N, typename Scalar = double>
    class RevoluteChainWithRotor : public SerialChain<Scalar>
    {
    public:
        typedef typename ClusterJoints::RevoluteWithRotor<Scalar> RevoluteWithRotor;
        typedef typename ClusterJoints::GearedTransmissionModule<Scalar> TransmissionModule;

        RevoluteChainWithRotor(bool random_parameters = true)
            : SerialChain<Scalar>(random_parameters) {}

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
                                 const std::string link_name) const;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_ROTOR_H
