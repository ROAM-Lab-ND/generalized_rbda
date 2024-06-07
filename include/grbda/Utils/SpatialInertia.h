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
#include "urdf_model/link.h"

namespace grbda
{

  // Conventional inertial parameters: mass, center of mass, and 3x3 rotational inertia about COM
  template <typename Scalar = double>
  struct InertialParams
  {
    Scalar m = 0;                           // mass
    Vec3<Scalar> c = Vec3<Scalar>::Zero();  // vector from body frame to COM
    Mat3<Scalar> Ic = Mat3<Scalar>::Zero(); // rotational inertia about COM
  };

  // Inertial parameters that the dynamics are linear with respect to:
  // mass, first mass moment, and 3x3 rotational inertia about the coordinate frame origin
  template <typename Scalar = double>
  struct LinearInertialParams
  {
    Scalar m = 0;                          // mass
    Vec3<Scalar> h = Vec3<Scalar>::Zero(); // first mass moment
    Mat3<Scalar> I = Mat3<Scalar>::Zero(); // rotational inertia about coordinate frame origin

    LinearInertialParams &operator=(const InertialParams<Scalar> &ip)
    {
      m = ip.m;
      h = ip.m * ip.c;
      I = ip.Ic + ip.m * ori::vectorToSkewMat(ip.c) * ori::vectorToSkewMat(ip.c).transpose();
      return *this;
    }

    LinearInertialParams &operator=(const Eigen::Vector<Scalar, 10> &other)
    {
      m = other(0);
      h = other.template segment<3>(1);
      I << other(4), other(5), other(6), other(5), other(7), other(8), other(6), other(8), other(9);
      return *this;
    }

    Vec10<Scalar> toVector() const
    {
      Vec10<Scalar> v;
      v << m, h, I(0, 0), I(0, 1), I(0, 2), I(1, 1), I(1, 2), I(2, 2);
      return v;
    }

  };

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
     * Construct spatial inertia from struct containting mass, first mass moment, and 3x3 rotational
     * inertia about the coordinate frame origin
     */
    SpatialInertia(const LinearInertialParams<T> &ip)
    {
      Mat3<T> hSkew = ori::vectorToSkewMat(ip.h);
      _inertia.template topLeftCorner<3, 3>() = ip.I;
      _inertia.template topRightCorner<3, 3>() = hSkew;
      _inertia.template bottomLeftCorner<3, 3>() = hSkew.transpose();
      _inertia.template bottomRightCorner<3, 3>() = ip.m * Mat3<T>::Identity();
    }

    /*!
     * Construct urdf Inertial
     */
    explicit SpatialInertia(const std::shared_ptr<const urdf::Inertial> &inertial)
    {
      T mass = inertial->mass;
      Vec3<T> COM = Vec3<T>(inertial->origin.position.x,
                            inertial->origin.position.y,
                            inertial->origin.position.z);
      Mat3<T> inertia;
      inertia.row(0) << inertial->ixx, inertial->ixy, inertial->ixz;
      inertia.row(1) << inertial->ixy, inertial->iyy, inertial->iyz;
      inertia.row(2) << inertial->ixz, inertial->iyz, inertial->izz;
      *this = SpatialInertia(mass, COM, inertia);
    }

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
    T getMass() const { return _inertia(5, 5); }

    /*!
     * Get center of mass location
     */
    Vec3<T> getCOM() const
    {
      T m = getMass();
      Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
      Vec3<T> com = ori::matToSkewVec(mcSkew) / m;
      return com;
    }

    /*!
     * Get 3x3 rotational inertia
     */
    Mat3<T> getInertiaTensor() const
    {
      T m = getMass();
      Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
      Mat3<T> I_rot = _inertia.template topLeftCorner<3, 3>() -
                      mcSkew * mcSkew.transpose() / m;
      return I_rot;
    }

    /*!
     * Get inertial parameters
     */
    InertialParams<T> getInertialParams() const
    {
      InertialParams<T> ip;
      ip.m = getMass();
      ip.c = getCOM();
      ip.Ic = getInertiaTensor();
      return ip;
    }

    /*!
     * Get linear inertial parameters
     */
    LinearInertialParams<T> getLinearInertialParams() const
    {
      LinearInertialParams<T> ip;
      ip.m = getMass();
      ip.h = ori::matToSkewVec(_inertia.template topRightCorner<3, 3>());
      ip.I = _inertia.template topLeftCorner<3, 3>();
      return ip;
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
