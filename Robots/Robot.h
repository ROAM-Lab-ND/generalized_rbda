#pragma once

#include "3rd-parties/CasadiGen/header/CasadiGen.h"
#include "Dynamics/ClusterTreeModel.h"

namespace grbda
{

  class Robot
  {
  public:
    Robot() {}
    virtual ~Robot() {}

    virtual ClusterTreeModel buildClusterTreeModel() const = 0;

    virtual DVec<double> forwardDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
      throw std::runtime_error("Not implemented");
    };

    virtual DVec<double> forwardDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
      throw std::runtime_error("Not implemented");
    };

    virtual DVec<double> forwardDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &tau) const
    {
      throw std::runtime_error("Not implemented");
    };
  };

} // namespace grbda
