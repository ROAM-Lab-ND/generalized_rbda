#ifndef GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_ROTOR_H
#define GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_ROTOR_H

#include "SerialChain.hpp"

namespace grbda
{

    template <size_t N>
    class RevoluteChainWithRotor : public SerialChain
    {
    public:
        RevoluteChainWithRotor(bool random_parameters = true) : SerialChain(random_parameters) {}

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
        ClusterTreeModel buildRandomClusterTreeModel() const override;
        ClusterTreeModel buildUniformClusterTreeModel() const override;

        void appendContactPoints(ClusterTreeModel &model, const int i,
                                 const std::string link_name) const;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_REVOLUTE_CHAIN_WITH_ROTOR_H
