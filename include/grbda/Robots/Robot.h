#ifndef GRBDA_ROBOT_H
#define GRBDA_ROBOT_H

#include "grbda/Codegen/CasadiGen.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

  template <typename Scalar>
  class Robot
  {
  public:
    Robot() {}
    virtual ~Robot() {}

    virtual ClusterTreeModel<Scalar> buildClusterTreeModel() const = 0;

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

    virtual DVec<double> inverseDynamics(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
      throw std::runtime_error("Not implemented");
    };

    virtual DVec<double> inverseDynamicsReflectedInertia(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
      throw std::runtime_error("Not implemented");
    };

    virtual DVec<double> inverseDynamicsReflectedInertiaDiag(
        const DVec<double> &y, const DVec<double> &yd, const DVec<double> &ydd) const
    {
      throw std::runtime_error("Not implemented");
    };
  };

} // namespace grbda

#endif // GRBDA_ROBOT_H
