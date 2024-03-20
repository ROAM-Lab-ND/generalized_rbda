/*! @file OrientationTools.h
 *  @brief Utility functions for 3D rotations
 *
 *  This file contains rotation utilities.  We generally use "coordinate
 * transformations" as opposed to the displacement transformations that are
 * commonly found in graphics.  To describe the orientation of a body, we use a
 * rotation matrix which transforms from world to body coordinates. This is the
 * transpose of the matrix which would rotate the body itself into the correct
 * orientation.
 *
 *  This follows the convention of Roy Featherstone's excellent book, Rigid Body
 * Dynamics Algorithms and the spatial_v2 MATLAB library that comes with it.
 * Note that we don't use the spatial_v2 convention for quaternions!
 */

#ifndef GRBDA_ORIENTATION_TOOLS_H
#define GRBDA_ORIENTATION_TOOLS_H

#include <cmath>
#include <iostream>
#include <type_traits>
#include "cppTypes.h"
#include "Utilities.h"
#include "grbda/Urdf/pose.h"

namespace grbda
{

  namespace ori
  {

    static constexpr double quaternionDerivativeStabilization = 0.1;

    enum class CoordinateAxis
    {
      X,
      Y,
      Z
    };

    /*!
     * Compute rotation matrix for coordinate transformation. Note that
     * coordinateRotation(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
     * this transforms into a frame rotated by .1 radians!.
     */
    template <typename T>
    Mat3<T> coordinateRotation(CoordinateAxis axis, T theta)
    {
      T s = sin(theta);
      T c = cos(theta);

      Mat3<T> R;

      if (axis == CoordinateAxis::X)
      {
        R << 1, 0, 0, 0, c, s, 0, -s, c;
      }
      else if (axis == CoordinateAxis::Y)
      {
        R << c, 0, -s, 0, 1, 0, s, 0, c;
      }
      else if (axis == CoordinateAxis::Z)
      {
        R << c, s, 0, -s, c, 0, 0, 0, 1;
      }

      return R;
    }

    inline CoordinateAxis urdfAxisToCoordinateAxis(const urdf::Vector3 &axis)
    {
      if (axis.norm() != 1)
      {
        throw std::runtime_error("Error: Joint axis must be a unit vector");
      }

      if (axis.x == 1 || axis.x == -1)
      {
        return CoordinateAxis::X;
      }
      else if (axis.y == 1 || axis.y == -1)
      {
        return CoordinateAxis::Y;
      }
      else if (axis.z == 1 || axis.z == -1)
      {
        return CoordinateAxis::Z;
      }
      else
      {
        throw std::runtime_error("Error: Joint axis not defined");
      }
    }

    inline CoordinateAxis randomCoordinateAxis()
    {
      const int x = rand() % 3;
      if (x == 0)
        return CoordinateAxis::X;
      else if (x == 1)
        return CoordinateAxis::Y;
      else
        return CoordinateAxis::Z;
    }

    template <typename T>
    Mat3<typename T::Scalar> crossMatrix(const Eigen::MatrixBase<T> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "must have 3x1 vector");
      Mat3<typename T::Scalar> m;
      m << 0, -v(2), v(1),
          v(2), 0, -v(0),
          -v(1), v(0), 0;
      return m;
    }

    /*!
     * Go from rpy to rotation matrix.
     */
    template <typename T>
    Mat3<typename T::Scalar> rpyToRotMat(const Eigen::MatrixBase<T> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "must have 3x1 vector");
      Mat3<typename T::Scalar> m = coordinateRotation(CoordinateAxis::X, v[0]) *
                                   coordinateRotation(CoordinateAxis::Y, v[1]) *
                                   coordinateRotation(CoordinateAxis::Z, v[2]);
      return m;
    }

    /*!
     * Convert a 3x1 vector to a skew-symmetric 3x3 matrix
     */
    template <typename T>
    Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "Must have 3x1 matrix");
      Mat3<typename T::Scalar> m;
      m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
      return m;
    }

    /*!
     * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector
     */
    template <typename T>
    Vec3<typename T::Scalar> matToSkewVec(const Eigen::MatrixBase<T> &m)
    {
      static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                    "Must have 3x3 matrix");
      return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0),
                                            (m(1, 0) - m(0, 1)));
    }

    /*!
     * Convert a coordinate transformation matrix to an orientation quaternion.
     */
    template <typename T>
    inline Quat<T> rotationMatrixToQuaternion(const Mat3<T> &r1)
    {
      Quat<T> q;
      Mat3<T> r = r1.transpose();
      T tr = r.trace();

      if (tr > 0.0)
      {
        T S = sqrt(tr + 1.0) * 2.0;
        q(0) = 0.25 * S;
        q(1) = (r(2, 1) - r(1, 2)) / S;
        q(2) = (r(0, 2) - r(2, 0)) / S;
        q(3) = (r(1, 0) - r(0, 1)) / S;
      }
      else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2)))
      {
        T S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
        q(0) = (r(2, 1) - r(1, 2)) / S;
        q(1) = 0.25 * S;
        q(2) = (r(0, 1) + r(1, 0)) / S;
        q(3) = (r(0, 2) + r(2, 0)) / S;
      }
      else if (r(1, 1) > r(2, 2))
      {
        T S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
        q(0) = (r(0, 2) - r(2, 0)) / S;
        q(1) = (r(0, 1) + r(1, 0)) / S;
        q(2) = 0.25 * S;
        q(3) = (r(1, 2) + r(2, 1)) / S;
      }
      else
      {
        T S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
        q(0) = (r(1, 0) - r(0, 1)) / S;
        q(1) = (r(0, 2) + r(2, 0)) / S;
        q(2) = (r(1, 2) + r(2, 1)) / S;
        q(3) = 0.25 * S;
      }
      return q;
    }

    template <>
    inline Quat<casadi::SX> rotationMatrixToQuaternion(const Mat3<casadi::SX> &r1)
    {
      Quat<casadi::SX> q;
      Mat3<casadi::SX> r = r1.transpose();
      casadi::SX tr = r.trace();

      casadi::SX cond1 = tr > 0.;
      casadi::SX cond2 = (r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2));
      casadi::SX cond3 = r(1, 1) > r(2, 2);

      casadi::SX radicand1 = tr + 1.0;
      casadi::SX radicand2 = 1.0 + r(0, 0) - r(1, 1) - r(2, 2);
      casadi::SX radicand3 = 1.0 + r(1, 1) - r(0, 0) - r(2, 2);
      casadi::SX radicand4 = 1.0 + r(2, 2) - r(0, 0) - r(1, 1);

      casadi::SX S = casadi::SX::if_else(cond1, grbda::sqrt(radicand1) * 2.0,
                     casadi::SX::if_else(cond2, grbda::sqrt(radicand2) * 2.0,
                     casadi::SX::if_else(cond3, grbda::sqrt(radicand3) * 2.0,
                                                grbda::sqrt(radicand4) * 2.0)));

      q(0) = casadi::SX::if_else(cond1, 0.25 * S,
             casadi::SX::if_else(cond2, (r(2, 1) - r(1, 2)) / S,
             casadi::SX::if_else(cond3, (r(0, 2) - r(2, 0)) / S,
                                        (r(1, 0) - r(0, 1)) / S)));

      q(1) = casadi::SX::if_else(cond1, (r(2, 1) - r(1, 2)) / S,
             casadi::SX::if_else(cond2, 0.25 * S,
             casadi::SX::if_else(cond3, (r(1, 0) + r(0, 1)) / S,
                                        (r(0, 2) + r(2, 0)) / S)));

      q(2) = casadi::SX::if_else(cond1, (r(0, 2) - r(2, 0)) / S,
             casadi::SX::if_else(cond2, (r(1, 0) + r(0, 1)) / S,
             casadi::SX::if_else(cond3, 0.25 * S,
                                        (r(1, 2) + r(2, 1)) / S)));

      q(3) = casadi::SX::if_else(cond1, (r(1, 0) - r(0, 1)) / S,
             casadi::SX::if_else(cond2, (r(0, 2) + r(2, 0)) / S,
             casadi::SX::if_else(cond3, (r(1, 2) + r(2, 1)) / S,
                                        0.25 * S)));

      return q;
    }

    /*!
     * Convert a quaternion to a rotation matrix.  This matrix represents a
     * coordinate transformation into the frame which has the orientation specified
     * by the quaternion
     */
    template <typename T>
    Mat3<typename T::Scalar> quaternionToRotationMatrix(
        const Eigen::MatrixBase<T> &q)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                    "Must have 4x1 quat");
      typename T::Scalar e0 = q(0);
      typename T::Scalar e1 = q(1);
      typename T::Scalar e2 = q(2);
      typename T::Scalar e3 = q(3);

      Mat3<typename T::Scalar> R;

      R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2),
          2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
          2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2);
      R.transposeInPlace();
      return R;
    }

    /*!
     * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
     * angles in (roll, pitch, yaw).
     */
    template <typename T>
    Vec3<typename T::Scalar> quatToRPY(const Eigen::MatrixBase<T> &q)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                    "Must have 4x1 quat");
      Vec3<typename T::Scalar> rpy;
      typename T::Scalar as = min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
      rpy(2) =
          atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
      rpy(1) = asin(as);
      rpy(0) =
          atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
      return rpy;
    }

    template <typename T>
    Quat<typename T::Scalar> rpyToQuat(const Eigen::MatrixBase<T> &rpy)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "Must have 3x1 vec");
      Mat3<typename T::Scalar> R = rpyToRotMat(rpy);
      Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
      return q;
    }

    /*!
     * Convert a quaternion to so3.
     */
    template <typename T>
    Vec3<typename T::Scalar> quatToso3(const Eigen::MatrixBase<T> &q)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                    "Must have 4x1 quat");
      Vec3<typename T::Scalar> so3;
      typename T::Scalar theta = 2. * acos(q[0]);
      if (theta * theta < 0.000000001)
      {
        so3.setZero();
      }
      else
      {
        so3[0] = theta * q[1] / sin(theta / 2.);
        so3[1] = theta * q[2] / sin(theta / 2.);
        so3[2] = theta * q[3] / sin(theta / 2.);
      }
      return so3;
    }

    template <typename T>
    Vec3<typename T::Scalar> rotationMatrixToRPY(const Eigen::MatrixBase<T> &R)
    {
      static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                    "Must have 3x3 matrix");
      Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
      Vec3<typename T::Scalar> rpy = quatToRPY(q);
      return rpy;
    }
    /*!
     * Quaternion derivative calculation, like rqd(q, omega) in MATLAB.
     * the omega is expressed in body frame
     * @param q
     * @param omega
     * @return
     */
    template <typename T, typename T2>
    Quat<typename T::Scalar> quatDerivative(const Eigen::MatrixBase<T> &q,
                                            const Eigen::MatrixBase<T2> &omega)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                    "Must have 4x1 quat");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 omega");
      // first case in rqd
      Mat4<typename T::Scalar> Q;
      Q << q[0], -q[1], -q[2], -q[3], q[1], q[0], -q[3], q[2], q[2], q[3], q[0],
          -q[1], q[3], -q[2], q[1], q[0];

      Quat<typename T::Scalar> qq(
          quaternionDerivativeStabilization * omega.norm() * (1 - q.norm()),
          omega[0], omega[1], omega[2]);
      Quat<typename T::Scalar> dq = 0.5 * Q * qq;
      return dq;
    }

    /*!
     * Take the product of two quaternions
     */
    template <typename T>
    Quat<typename T::Scalar> quatProduct(const Eigen::MatrixBase<T> &q1,
                                         const Eigen::MatrixBase<T> &q2)
    {
      typename T::Scalar r1 = q1[0];
      typename T::Scalar r2 = q2[0];
      Vec3<typename T::Scalar> v1(q1[1], q1[2], q1[3]);
      Vec3<typename T::Scalar> v2(q2[1], q2[2], q2[3]);

      typename T::Scalar r = r1 * r2 - v1.dot(v2);
      Vec3<typename T::Scalar> v = r1 * v2 + r2 * v1 + v1.cross(v2);
      Quat<typename T::Scalar> q(r, v[0], v[1], v[2]);
      return q;
    }

    /*!
     * Compute new quaternion given:
     * @param quat The old quaternion
     * @param omega The angular velocity (IN INERTIAL COORDINATES!)
     * @param dt The timestep
     * @return
     */
    template <typename T, typename T2, typename T3>
    Quat<typename T::Scalar> integrateQuat(const Eigen::MatrixBase<T> &quat,
                                           const Eigen::MatrixBase<T2> &omega,
                                           T3 dt)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                    "Must have 4x1 quat");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 omega");
      Vec3<typename T::Scalar> axis;
      typename T::Scalar ang = omega.norm();
      if (ang > 0)
      {
        axis = omega / ang;
      }
      else
      {
        axis = Vec3<typename T::Scalar>(1, 0, 0);
      }

      ang *= dt;
      Vec3<typename T::Scalar> ee = sin(ang / 2) * axis;
      Quat<typename T::Scalar> quatD(cos(ang / 2), ee[0], ee[1], ee[2]);

      Quat<typename T::Scalar> quatNew = quatProduct(quatD, quat);
      quatNew = quatNew / quatNew.norm();
      return quatNew;
    }

    template <>
    inline Quat<casadi::SX> integrateQuat(const Eigen::MatrixBase<Quat<casadi::SX>> &quat,
                                          const Eigen::MatrixBase<Vec3<casadi::SX>> &omega,
                                          casadi::SX dt)
    {
      Quat<casadi::SX> quat_new = quat + 0.5 * quatProduct(Quat<casadi::SX>(0, omega[0], omega[1], omega[2]), quat) * dt;
      return quat_new / quat_new.norm();
    }

    /*!
     * Compute new quaternion given:
     * @param quat The old quaternion
     * @param omega The angular velocity (IN BODY COORDINATES!)
     * @param dt The timestep
     * @return
     */
    template <typename T, typename T2, typename T3>
    Quat<typename T::Scalar> integrateQuatImplicit(
        const Eigen::MatrixBase<T> &quat, const Eigen::MatrixBase<T2> &omega,
        T3 dt)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                    "Must have 4x1 quat");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 omega");
      Vec3<typename T::Scalar> axis;
      typename T::Scalar ang = omega.norm();
      if (ang > 0)
      {
        axis = omega / ang;
      }
      else
      {
        axis = Vec3<typename T::Scalar>(1, 0, 0);
      }

      ang *= dt;
      Vec3<typename T::Scalar> ee = sin(ang / 2) * axis;
      Quat<typename T::Scalar> quatD(cos(ang / 2), ee[0], ee[1], ee[2]);

      Quat<typename T::Scalar> quatNew = quatProduct(quat, quatD);
      quatNew = quatNew / quatNew.norm();
      return quatNew;
    }

    template <>
    inline Quat<casadi::SX> integrateQuatImplicit(const Eigen::MatrixBase<Quat<casadi::SX>> &quat,
                                                  const Eigen::MatrixBase<Vec3<casadi::SX>> &omega,
                                                  casadi::SX dt)
    {
      Quat<casadi::SX> quat_new = quat + 0.5 * quatProduct(quat, Quat<casadi::SX>(0, omega[0], omega[1], omega[2])) * dt;
      return quat_new / quat_new.norm();
    }

    template <typename T>
    void quaternionToso3(const Quat<T> quat, Vec3<T> &so3)
    {
      so3[0] = quat[1];
      so3[1] = quat[2];
      so3[2] = quat[3];

      T theta =
          2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));

      if (fabs(theta) < 0.0000001)
      {
        so3.setZero();
        return;
      }
      so3 /= sin(theta / 2.0);
      so3 *= theta;
    }
    template <typename T>
    Quat<T> so3ToQuat(Vec3<T> &so3)
    {
      Quat<T> quat;

      T theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);

      if (fabs(theta) < 1.e-6)
      {
        quat.setZero();
        quat[0] = 1.;
        return quat;
      }
      quat[0] = cos(theta / 2.);
      quat[1] = so3[0] / theta * sin(theta / 2.);
      quat[2] = so3[1] / theta * sin(theta / 2.);
      quat[3] = so3[2] / theta * sin(theta / 2.);
      return quat;
    }

    /*!
     * Go from rotation matrix to rpy. CHARLES
     */
    template <typename T>
    Vec3<typename T::Scalar> RotMatToRPY(const Eigen::MatrixBase<T> &r1)
    {
      static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                    "Must have 3x3 matrix");

      Quat<typename T::Scalar> q = rotationMatrixToQuaternion(r1);
      Vec3<typename T::Scalar> rpy;

      rpy = quatToRPY(q);
      return rpy;
    }

  } // namespace ori

} // namespace grbda

#endif // GRBDA_ORIENTATION_TOOLS_H
