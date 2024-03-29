/*! @file SpatialInertia.h
 *  @brief Class representing spatial inertia tensors
 *
 */

#ifndef GRDBA_SPATIAL_INERTIA_H
#define GRDBA_SPATIAL_INERTIA_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "OrientationTools.h"
#include "Spatial.h"

namespace grbda
{
  /*!
   * Representation of Rigid Body Inertia as a 6x6 Spatial Inertia Tensor
   */
  template <typename T>
  class SpatialInertia
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * Construct spatial inertia from mass, center of mass, and 3x3 rotational inertia
     */
    SpatialInertia(T mass, const Vec3<T> &com, const Mat3<T> &inertia)
    {
      Mat3<T> cSkew = ori::vectorToSkewMat(com);
      _inertia.template topLeftCorner<3, 3>() =
          inertia + mass * cSkew * cSkew.transpose();
      _inertia.template topRightCorner<3, 3>() = mass * cSkew;
      _inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
      _inertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
    }

    /*!
     * Construct spatial inertia from 6x6 matrix
     */
    explicit SpatialInertia(const Mat6<T> &inertia) { _inertia = inertia; }

    /*!
     * If no argument is given, zero.
     */
    SpatialInertia() { _inertia = Mat6<T>::Zero(); }

    /*!
     * Construct spatial inertia from pseudo-inertia. This is described in
     * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
     *   Identification: A Statistical Perspective on the Mass Distribution, by
     *   Wensing, Kim, Slotine
     * @param P
     */
    explicit SpatialInertia(const Mat4<T> &P)
    {
      Mat6<T> I;
      T m = P(3, 3);
      Vec3<T> h = P.template topRightCorner<3, 1>();
      Mat3<T> E = P.template topLeftCorner<3, 3>();
      Mat3<T> Ibar = E.trace() * Mat3<T>::Identity() - E;
      I.template topLeftCorner<3, 3>() = Ibar;
      I.template topRightCorner<3, 3>() = ori::vectorToSkewMat(h);
      I.template bottomLeftCorner<3, 3>() = ori::vectorToSkewMat(h).transpose();
      I.template bottomRightCorner<3, 3>() = m * Mat3<T>::Identity();
      _inertia = I;
    }

    static SpatialInertia<T> createRandomInertia(T scaling = 1.0)
    {
      const T mass = scaling * ((T)rand() / (T)RAND_MAX);
      const Vec3<T> com = scaling * Vec3<T>::Random();
      const Mat3<T> rand_3x3 = Mat3<T>::Random();
      const Mat3<T> inertia = scaling * (rand_3x3 * rand_3x3.transpose());
      return SpatialInertia<T>(mass, com, inertia);
    }

    /*!
     * Get 6x6 spatial inertia
     */
    const Mat6<T> &getMatrix() const { return _inertia; }

    /*!
     * Get mass
     */
    T getMass() { return _inertia(5, 5); }

    /*!
     * Get center of mass location
     */
    Vec3<T> getCOM()
    {
      T m = getMass();
      Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
      Vec3<T> com = ori::matToSkewVec(mcSkew) / m;
      return com;
    }

    /*!
     * Get 3x3 rotational inertia
     */
    Mat3<T> getInertiaTensor()
    {
      T m = getMass();
      Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
      Mat3<T> I_rot = _inertia.template topLeftCorner<3, 3>() -
                      mcSkew * mcSkew.transpose() / m;
      return I_rot;
    }

    /*!
     * Convert to 4x4 pseudo-inertia matrix.  This is described in
     * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
     *   Identification: A Statistical Perspective on the Mass Distribution, by
     *   Wensing, Kim, Slotine
     */
    Mat4<T> getPseudoInertia() const
    {
      Vec3<T> h = ori::matToSkewVec(_inertia.template topRightCorner<3, 3>());
      Mat3<T> Ibar = _inertia.template topLeftCorner<3, 3>();
      T m = _inertia(5, 5);
      Mat4<T> P;
      P.template topLeftCorner<3, 3>() =
          0.5 * Ibar.trace() * Mat3<T>::Identity() - Ibar;
      P.template topRightCorner<3, 1>() = h;
      P.template bottomLeftCorner<1, 3>() = h.transpose();
      P(3, 3) = m;
      return P;
    }

    /*!
     * Flip inertia matrix around an axis.  This isn't efficient, but it works!
     */
    SpatialInertia flipAlongAxis(ori::CoordinateAxis axis) const
    {
      Mat4<T> P = getPseudoInertia();
      Mat4<T> X = Mat4<T>::Identity();
      if (axis == ori::CoordinateAxis::X)
        X(0, 0) = -1;
      else if (axis == ori::CoordinateAxis::Y)
        X(1, 1) = -1;
      else if (axis == ori::CoordinateAxis::Z)
        X(2, 2) = -1;
      P = X * P * X;
      return SpatialInertia(P);
    }

  private:
    Mat6<T> _inertia;
  };

} // namespace grbda

#endif // GRBDA_SPATIAL_INERTIA_H
