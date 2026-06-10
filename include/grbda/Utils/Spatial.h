/*! @file Spatial.h
 *  @brief Utility functions for manipulating spatial quantities
 *
 *  This file contains functions for working with spatial vectors and
 * transformation matrices.
 */

#ifndef GRBDA_SPATIAL_H
#define GRBDA_SPATIAL_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "SpatialTransforms.h"

namespace grbda
{

  namespace spatial
  {

    enum class JointType
    {
      Prismatic,
      Revolute,
      FloatingBase,
      Nothing
    };

    /*!
     * Calculate the spatial coordinate transform from A to B where B is rotate by
     * theta about axis.
     */
    template <typename T>
    Transform<T> rotation(ori::CoordinateAxis axis, T theta)
    {
      RotMat<T> E = coordinateRotation(axis, theta);
      return Transform<T>(E);
    }

    template <typename T>
    Transform<T> randomSpatialRotation()
    {
      Vec3<T> r = Vec3<T>::Random();
      Mat3<T> E = ori::rpyToRotMat(Vec3<T>::Random());
      return Transform<T>(E, r);
    }

    /*!
     * Compute the spatial motion cross product matrix. Prefer motionCrossProduct when possible.
     */
    template <typename T>
    Mat6<typename T::Scalar> motionCrossMatrix(const Eigen::MatrixBase<T> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
      Mat6<typename T::Scalar> m;
      m << 0, -v(2), v(1), 0, 0, 0,
          v(2), 0, -v(0), 0, 0, 0,
          -v(1), v(0), 0, 0, 0, 0,
          0, -v(5), v(4), 0, -v(2), v(1),
          v(5), 0, -v(3), v(2), 0, -v(0),
          -v(4), v(3), 0, -v(1), v(0), 0;
      return m;
    }

    /*!
     * Compute the spatial motion cross product matrix. Prefer generalMotionCrossProduct when possible.
     *
     * This is a general formulation to deal with the the ability of aggregate bodies to have
     * 6N dimensional spatial velocities
     */
    template <typename T>
    DMat<T> generalMotionCrossMatrix(const DVec<T> &v)
    {
      const int n = v.rows();
      if (n == 6)
        return motionCrossMatrix(v.template head<6>());
      else if (n % 6 == 0)
      {
        DMat<T> m = DMat<T>::Zero(n, n);
        for (int i = 0; i < (n / 6); i++)
          m.template block<6, 6>(6 * i, 6 * i) = motionCrossMatrix(v.template segment<6>(6 * i));
        return m;
      }
      else
        throw std::runtime_error("Invalid number of rows provided to General Motion Cross Matrix");
    }

    /*!
     * Compute spatial force cross product matrix. Prefer forceCrossProduct when possible
     */
    template <typename T>
    auto forceCrossMatrix(const Eigen::MatrixBase<T> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
      Mat6<typename T::Scalar> f;
      f << 0, -v(2), v(1), 0, -v(5), v(4), v(2), 0, -v(0), v(5), 0, -v(3), -v(1),
          v(0), 0, -v(4), v(3), 0, 0, 0, 0, 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0),
          0, 0, 0, -v(1), v(0), 0;
      return f;
    }

    /*!
     * Compute spatial force cross product matrix. Prefer generalForceCrossProduct when possible
     *
     * This is a general formulation to deal with the the ability of aggregate bodies to have
     * 6N dimensional spatial forces
     */
    template <typename T>
    DMat<T> generalForceCrossMatrix(const DVec<T> &v)
    {
      const int n = v.rows();
      if (n == 6)
        return forceCrossMatrix(v.template head<6>());
      else if (n % 6 == 0)
      {
        DMat<T> f = DMat<T>::Zero(n, n);
        for (int i = 0; i < (n / 6); i++)
          f.template block<6, 6>(6 * i, 6 * i) = forceCrossMatrix(v.template segment<6>(6 * i));
        return f;
      }
      else
        throw std::runtime_error("Invalid number of rows provided to General Force Cross Matrix");
    }

    /*!
     * Compute spatial motion cross product.  Faster than the matrix multiplication
     * version
     */
    template <typename T>
    auto motionCrossProduct(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
      SVec<typename T::Scalar> mv;
      mv << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2),
          a(0) * b(1) - a(1) * b(0),
          a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1),
          a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
          a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
      return mv;
    }

    /*!
     * Compute spatial motion cross product into a pre-allocated output vector.
     */
    template <typename T>
    void generalMotionCrossProduct(const DVec<T> &a, const DVec<T> &b, DVec<T> &out)
    {
      const int n = a.rows();
      if (n != b.rows())
        throw std::runtime_error("General Motion Cross Product requires vectors of the same size");

      if (n == 6)
        out = motionCrossProduct(a.template head<6>(), b.template head<6>());
      else if (n % 6 == 0)
      {
        out.setZero(n);
        for (int i = 0; i < n / 6; i++)
          out.template segment<6>(6 * i) = motionCrossProduct(a.template segment<6>(6 * i),
                                                              b.template segment<6>(6 * i));
      }
      else
        throw std::runtime_error("Invalid number of rows provided to General Motion Cross Product");
    }

    /*!
     * Compute motion cross matrix times a matrix: out = crm(v) * M
     * Writes directly into the pre-allocated output matrix to avoid heap allocation.
     */
    template <typename Scalar>
    void motionCrossTimesMatrix(const DVec<Scalar> &v, const DMat<Scalar> &M, DMat<Scalar> &out)
    {
      const int n = v.rows();
      const int cols = M.cols();

      if (n == 6)
      {
        out.resize(6, cols);
        for (int c = 0; c < cols; ++c)
        {
          out(0, c) = -v(2)*M(1,c) + v(1)*M(2,c);
          out(1, c) =  v(2)*M(0,c) - v(0)*M(2,c);
          out(2, c) = -v(1)*M(0,c) + v(0)*M(1,c);
          out(3, c) = -v(5)*M(1,c) + v(4)*M(2,c) - v(2)*M(4,c) + v(1)*M(5,c);
          out(4, c) =  v(5)*M(0,c) - v(3)*M(2,c) + v(2)*M(3,c) - v(0)*M(5,c);
          out(5, c) = -v(4)*M(0,c) + v(3)*M(1,c) - v(1)*M(3,c) + v(0)*M(4,c);
        }
      }
      else if (n % 6 == 0)
      {
        out.setZero(n, cols);
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          for (int c = 0; c < cols; ++c)
          {
            out(o+0, c) = -v(o+2)*M(o+1,c) + v(o+1)*M(o+2,c);
            out(o+1, c) =  v(o+2)*M(o+0,c) - v(o+0)*M(o+2,c);
            out(o+2, c) = -v(o+1)*M(o+0,c) + v(o+0)*M(o+1,c);
            out(o+3, c) = -v(o+5)*M(o+1,c) + v(o+4)*M(o+2,c) - v(o+2)*M(o+4,c) + v(o+1)*M(o+5,c);
            out(o+4, c) =  v(o+5)*M(o+0,c) - v(o+3)*M(o+2,c) + v(o+2)*M(o+3,c) - v(o+0)*M(o+5,c);
            out(o+5, c) = -v(o+4)*M(o+0,c) + v(o+3)*M(o+1,c) - v(o+1)*M(o+3,c) + v(o+0)*M(o+4,c);
          }
        }
      }
      else
      {
        throw std::runtime_error("Invalid dimension for motionCrossTimesMatrix");
      }
    }

    template <typename Scalar>
    DMat<Scalar> motionCrossTimesMatrix(const DVec<Scalar> &v, const DMat<Scalar> &M)
    {
      DMat<Scalar> out;
      motionCrossTimesMatrix(v, M, out);
      return out;
    }

    /*!
     * Compute motion cross matrix times a matrix and accumulate: out += crm(v) * M
     */
    template <typename Scalar>
    void addMotionCrossTimesMatrix(const DVec<Scalar> &v, const DMat<Scalar> &M, DMat<Scalar> &out)
    {
      const int n = v.rows();
      const int cols = M.cols();

      if (n == 6)
      {
        for (int c = 0; c < cols; ++c)
        {
          out(0, c) += -v(2)*M(1,c) + v(1)*M(2,c);
          out(1, c) +=  v(2)*M(0,c) - v(0)*M(2,c);
          out(2, c) += -v(1)*M(0,c) + v(0)*M(1,c);
          out(3, c) += -v(5)*M(1,c) + v(4)*M(2,c) - v(2)*M(4,c) + v(1)*M(5,c);
          out(4, c) +=  v(5)*M(0,c) - v(3)*M(2,c) + v(2)*M(3,c) - v(0)*M(5,c);
          out(5, c) += -v(4)*M(0,c) + v(3)*M(1,c) - v(1)*M(3,c) + v(0)*M(4,c);
        }
      }
      else if (n % 6 == 0)
      {
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          for (int c = 0; c < cols; ++c)
          {
            out(o+0, c) += -v(o+2)*M(o+1,c) + v(o+1)*M(o+2,c);
            out(o+1, c) +=  v(o+2)*M(o+0,c) - v(o+0)*M(o+2,c);
            out(o+2, c) += -v(o+1)*M(o+0,c) + v(o+0)*M(o+1,c);
            out(o+3, c) += -v(o+5)*M(o+1,c) + v(o+4)*M(o+2,c) - v(o+2)*M(o+4,c) + v(o+1)*M(o+5,c);
            out(o+4, c) +=  v(o+5)*M(o+0,c) - v(o+3)*M(o+2,c) + v(o+2)*M(o+3,c) - v(o+0)*M(o+5,c);
            out(o+5, c) += -v(o+4)*M(o+0,c) + v(o+3)*M(o+1,c) - v(o+1)*M(o+3,c) + v(o+0)*M(o+4,c);
          }
        }
      }
      else
      {
        throw std::runtime_error("Invalid dimension for addMotionCrossTimesMatrix");
      }
    }

    /*!
     * Compute matrix times motion cross matrix: M * crm(v)
     * This avoids building the full 6x6 cross-product matrix.
     * Result(i,j) = sum_k M(i,k) * crm(v)(k,j)
     */
    template <typename Scalar>
    DMat<Scalar> matrixTimesMotionCross(const DMat<Scalar> &M, const DVec<Scalar> &v)
    {
      const int rows = M.rows();
      const int n = v.rows();

      if (n == 6)
      {
        // crm(v) structure (from motionCrossMatrix):
        // [  0  -v2   v1   0    0    0  ]
        // [ v2   0   -v0   0    0    0  ]
        // [-v1  v0    0    0    0    0  ]
        // [  0  -v5   v4   0   -v2   v1 ]
        // [ v5   0   -v3  v2    0   -v0 ]
        // [-v4  v3    0  -v1   v0    0  ]
        // Column j of result = M * (column j of crm(v))
        DMat<Scalar> result(rows, 6);
        for (int r = 0; r < rows; ++r)
        {
          // Col 0 of crm: [0, v2, -v1, 0, v5, -v4]^T
          result(r, 0) = v(2)*M(r,1) - v(1)*M(r,2) + v(5)*M(r,4) - v(4)*M(r,5);
          // Col 1 of crm: [-v2, 0, v0, -v5, 0, v3]^T
          result(r, 1) = -v(2)*M(r,0) + v(0)*M(r,2) - v(5)*M(r,3) + v(3)*M(r,5);
          // Col 2 of crm: [v1, -v0, 0, v4, -v3, 0]^T
          result(r, 2) = v(1)*M(r,0) - v(0)*M(r,1) + v(4)*M(r,3) - v(3)*M(r,4);
          // Col 3 of crm: [0, 0, 0, 0, v2, -v1]^T
          result(r, 3) = v(2)*M(r,4) - v(1)*M(r,5);
          // Col 4 of crm: [0, 0, 0, -v2, 0, v0]^T
          result(r, 4) = -v(2)*M(r,3) + v(0)*M(r,5);
          // Col 5 of crm: [0, 0, 0, v1, -v0, 0]^T
          result(r, 5) = v(1)*M(r,3) - v(0)*M(r,4);
        }
        return result;
      }
      else if (n % 6 == 0 && rows % 6 == 0)
      {
        // Block diagonal case: M and crm(v) are both block-diagonal
        DMat<Scalar> result = DMat<Scalar>::Zero(rows, n);
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          for (int r = 0; r < 6; ++r)
          {
            const int rr = o + r;
            result(rr, o+0) = v(o+2)*M(rr,o+1) - v(o+1)*M(rr,o+2) + v(o+5)*M(rr,o+4) - v(o+4)*M(rr,o+5);
            result(rr, o+1) = -v(o+2)*M(rr,o+0) + v(o+0)*M(rr,o+2) - v(o+5)*M(rr,o+3) + v(o+3)*M(rr,o+5);
            result(rr, o+2) = v(o+1)*M(rr,o+0) - v(o+0)*M(rr,o+1) + v(o+4)*M(rr,o+3) - v(o+3)*M(rr,o+4);
            result(rr, o+3) = v(o+2)*M(rr,o+4) - v(o+1)*M(rr,o+5);
            result(rr, o+4) = -v(o+2)*M(rr,o+3) + v(o+0)*M(rr,o+5);
            result(rr, o+5) = v(o+1)*M(rr,o+3) - v(o+0)*M(rr,o+4);
          }
        }
        return result;
      }
      else
      {
        throw std::runtime_error("Invalid dimension for matrixTimesMotionCross");
      }
    }

    /*!
     * Compute crf(v)*I - I*crm(v) for spatial inertia I and velocity v.
     * Writes directly into the pre-allocated output matrix to avoid heap allocation.
     */
    template <typename Scalar>
    void spatialInertiaCrossTerms(const DMat<Scalar> &I, const DVec<Scalar> &v, DMat<Scalar> &out)
    {
      const int n = v.rows();

      if (n == 6)
      {
        out.resize(6, 6);
        for (int r = 0; r < 6; ++r)
        {
          for (int c = 0; c < 6; ++c)
          {
            Scalar crf_part;
            switch (r) {
              case 0: crf_part = -v(2)*I(1,c) + v(1)*I(2,c) - v(5)*I(4,c) + v(4)*I(5,c); break;
              case 1: crf_part =  v(2)*I(0,c) - v(0)*I(2,c) + v(5)*I(3,c) - v(3)*I(5,c); break;
              case 2: crf_part = -v(1)*I(0,c) + v(0)*I(1,c) - v(4)*I(3,c) + v(3)*I(4,c); break;
              case 3: crf_part = -v(2)*I(4,c) + v(1)*I(5,c); break;
              case 4: crf_part =  v(2)*I(3,c) - v(0)*I(5,c); break;
              case 5: crf_part = -v(1)*I(3,c) + v(0)*I(4,c); break;
              default: crf_part = Scalar(0); break;
            }

            Scalar crm_part;
            switch (c) {
              case 0: crm_part = v(2)*I(r,1) - v(1)*I(r,2) + v(5)*I(r,4) - v(4)*I(r,5); break;
              case 1: crm_part = -v(2)*I(r,0) + v(0)*I(r,2) - v(5)*I(r,3) + v(3)*I(r,5); break;
              case 2: crm_part = v(1)*I(r,0) - v(0)*I(r,1) + v(4)*I(r,3) - v(3)*I(r,4); break;
              case 3: crm_part = v(2)*I(r,4) - v(1)*I(r,5); break;
              case 4: crm_part = -v(2)*I(r,3) + v(0)*I(r,5); break;
              case 5: crm_part = v(1)*I(r,3) - v(0)*I(r,4); break;
              default: crm_part = Scalar(0); break;
            }

            out(r, c) = crf_part - crm_part;
          }
        }
      }
      else if (n % 6 == 0)
      {
        out.setZero(n, n);
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          for (int r = 0; r < 6; ++r)
          {
            for (int c = 0; c < 6; ++c)
            {
              const int rr = o + r;
              const int cc = o + c;

              Scalar crf_part;
              switch (r) {
                case 0: crf_part = -v(o+2)*I(o+1,cc) + v(o+1)*I(o+2,cc) - v(o+5)*I(o+4,cc) + v(o+4)*I(o+5,cc); break;
                case 1: crf_part =  v(o+2)*I(o+0,cc) - v(o+0)*I(o+2,cc) + v(o+5)*I(o+3,cc) - v(o+3)*I(o+5,cc); break;
                case 2: crf_part = -v(o+1)*I(o+0,cc) + v(o+0)*I(o+1,cc) - v(o+4)*I(o+3,cc) + v(o+3)*I(o+4,cc); break;
                case 3: crf_part = -v(o+2)*I(o+4,cc) + v(o+1)*I(o+5,cc); break;
                case 4: crf_part =  v(o+2)*I(o+3,cc) - v(o+0)*I(o+5,cc); break;
                case 5: crf_part = -v(o+1)*I(o+3,cc) + v(o+0)*I(o+4,cc); break;
                default: crf_part = Scalar(0); break;
              }

              Scalar crm_part;
              switch (c) {
                case 0: crm_part = v(o+2)*I(rr,o+1) - v(o+1)*I(rr,o+2) + v(o+5)*I(rr,o+4) - v(o+4)*I(rr,o+5); break;
                case 1: crm_part = -v(o+2)*I(rr,o+0) + v(o+0)*I(rr,o+2) - v(o+5)*I(rr,o+3) + v(o+3)*I(rr,o+5); break;
                case 2: crm_part = v(o+1)*I(rr,o+0) - v(o+0)*I(rr,o+1) + v(o+4)*I(rr,o+3) - v(o+3)*I(rr,o+4); break;
                case 3: crm_part = v(o+2)*I(rr,o+4) - v(o+1)*I(rr,o+5); break;
                case 4: crm_part = -v(o+2)*I(rr,o+3) + v(o+0)*I(rr,o+5); break;
                case 5: crm_part = v(o+1)*I(rr,o+3) - v(o+0)*I(rr,o+4); break;
                default: crm_part = Scalar(0); break;
              }

              out(rr, cc) = crf_part - crm_part;
            }
          }
        }
      }
      else
      {
        throw std::runtime_error("Invalid dimension for spatialInertiaCrossTerms");
      }
    }

    template <typename Scalar>
    DMat<Scalar> spatialInertiaCrossTerms(const DMat<Scalar> &I, const DVec<Scalar> &v)
    {
      DMat<Scalar> out;
      spatialInertiaCrossTerms(I, v, out);
      return out;
    }

    /*!
     * Compute force cross matrix times a matrix: crf(v) * M
     * This avoids building the full 6x6 cross-product matrix.
     */
    template <typename Scalar>
    DMat<Scalar> forceCrossTimesMatrix(const DVec<Scalar> &v, const DMat<Scalar> &M)
    {
      const int n = v.rows();
      const int cols = M.cols();

      if (n == 6)
      {
        // crf(v) structure (from forceCrossMatrix):
        // [  0  -v2   v1   0  -v5   v4 ]
        // [ v2   0   -v0  v5   0   -v3 ]
        // [-v1  v0    0  -v4  v3    0  ]
        // [  0   0    0    0  -v2   v1 ]
        // [  0   0    0   v2   0   -v0 ]
        // [  0   0    0  -v1  v0    0  ]
        // Row i of result = dot product of row i of crf(v) with column c of M
        DMat<Scalar> result(6, cols);
        for (int c = 0; c < cols; ++c)
        {
          // Row 0: [0, -v2, v1, 0, -v5, v4] . M(:,c)
          result(0, c) = -v(2)*M(1,c) + v(1)*M(2,c) - v(5)*M(4,c) + v(4)*M(5,c);
          // Row 1: [v2, 0, -v0, v5, 0, -v3] . M(:,c)
          result(1, c) =  v(2)*M(0,c) - v(0)*M(2,c) + v(5)*M(3,c) - v(3)*M(5,c);
          // Row 2: [-v1, v0, 0, -v4, v3, 0] . M(:,c)
          result(2, c) = -v(1)*M(0,c) + v(0)*M(1,c) - v(4)*M(3,c) + v(3)*M(4,c);
          // Row 3: [0, 0, 0, 0, -v2, v1] . M(:,c)
          result(3, c) = -v(2)*M(4,c) + v(1)*M(5,c);
          // Row 4: [0, 0, 0, v2, 0, -v0] . M(:,c)
          result(4, c) =  v(2)*M(3,c) - v(0)*M(5,c);
          // Row 5: [0, 0, 0, -v1, v0, 0] . M(:,c)
          result(5, c) = -v(1)*M(3,c) + v(0)*M(4,c);
        }
        return result;
      }
      else if (n % 6 == 0)
      {
        DMat<Scalar> result = DMat<Scalar>::Zero(n, cols);
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          for (int c = 0; c < cols; ++c)
          {
            result(o+0, c) = -v(o+2)*M(o+1,c) + v(o+1)*M(o+2,c) - v(o+5)*M(o+4,c) + v(o+4)*M(o+5,c);
            result(o+1, c) =  v(o+2)*M(o+0,c) - v(o+0)*M(o+2,c) + v(o+5)*M(o+3,c) - v(o+3)*M(o+5,c);
            result(o+2, c) = -v(o+1)*M(o+0,c) + v(o+0)*M(o+1,c) - v(o+4)*M(o+3,c) + v(o+3)*M(o+4,c);
            result(o+3, c) = -v(o+2)*M(o+4,c) + v(o+1)*M(o+5,c);
            result(o+4, c) =  v(o+2)*M(o+3,c) - v(o+0)*M(o+5,c);
            result(o+5, c) = -v(o+1)*M(o+3,c) + v(o+0)*M(o+4,c);
          }
        }
        return result;
      }
      else
      {
        throw std::runtime_error("Invalid dimension for forceCrossTimesMatrix");
      }
    }

    /*!
     * Compute spatial force cross product.  Faster than the matrix multiplication
     * version
     */
    template <typename T>
    auto forceCrossProduct(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
      SVec<typename T::Scalar> fv;
      fv << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4),
          b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
          b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3),
          b(5) * a(1) - b(4) * a(2), b(3) * a(2) - b(5) * a(0),
          b(4) * a(0) - b(3) * a(1);
      return fv;
    }

    /*!
     * Compute spatial force cross product, accumulating into a pre-allocated output vector.
     */
    template <typename T>
    void addGeneralForceCrossProduct(const DVec<T> &a, const DVec<T> &b, DVec<T> &out)
    {
      const int n = a.rows();
      if (n != b.rows())
        throw std::runtime_error("General Force Cross Product requires vectors of the same size");

      if (n == 6)
        out += forceCrossProduct(a.template head<6>(), b.template head<6>());
      else if (n % 6 == 0)
      {
        for (int i = 0; i < n / 6; i++)
          out.template segment<6>(6 * i) += forceCrossProduct(a.template segment<6>(6 * i),
                                                              b.template segment<6>(6 * i));
      }
      else
        throw std::runtime_error("Invalid number of rows provided to General Force Cross Product");
    }

    template <typename T>
    DVec<T> generalForceCrossProduct(const DVec<T> &a, const DVec<T> &b)
    {
      const int n = a.rows();
      DVec<T> fv = DVec<T>::Zero(n);
      addGeneralForceCrossProduct(a, b, fv);
      return fv;
    }

    /*!
     * Compute swapped force cross matrix.
     */
    template <typename T>
    auto swappedForceCrossMatrix(const Eigen::MatrixBase<T> &v)
    {
      Mat6<typename T::Scalar> f;
      f << 0, v(2), -v(1), 0, v(5), -v(4),
          -v(2), 0, v(0), -v(5), 0, v(3),
          v(1), -v(0), 0, v(4), -v(3), 0,
          0, v(5), -v(4), 0, 0, 0,
          -v(5), 0, v(3), 0, 0, 0,
          v(4), -v(3), 0, 0, 0, 0;

      return f;
    }

    /*!
     * Compute swapped force cross matrix. Generalized version for multi-body clusters
     *
     * This is a general formulation to deal with the ability of aggregate bodies to have
     * 6N dimensional spatial forces
     */
    template <typename T>
    DMat<T> generalSwappedForceCrossMatrix(const DVec<T> &v)
    {
      const int n = v.rows();
      if (n == 6)
        return swappedForceCrossMatrix(v.template head<6>());
      else if (n % 6 == 0)
      {
        DMat<T> f = DMat<T>::Zero(n, n);
        for (int i = 0; i < (n / 6); i++)
          f.template block<6, 6>(6 * i, 6 * i) = swappedForceCrossMatrix(v.template segment<6>(6 * i));
        return f;
      }
      else
        throw std::runtime_error("Invalid number of rows provided to General Swapped Force Cross Matrix");
    }

    /*!
     * Add swapped force cross matrix to an existing matrix in-place: M += icrf(v)
     * This avoids allocating a temporary matrix for multi-body clusters.
     */
    template <typename Scalar>
    void addSwappedForceCrossMatrixInPlace(DMat<Scalar> &M, const DVec<Scalar> &v)
    {
      const int n = v.rows();
      if (n == 6)
      {
        // icrf(v) structure:
        // [  0   v2  -v1   0   v5  -v4 ]
        // [ -v2   0   v0  -v5   0   v3 ]
        // [  v1  -v0   0   v4  -v3   0 ]
        // [  0   v5  -v4   0    0    0 ]
        // [ -v5   0   v3   0    0    0 ]
        // [  v4  -v3   0   0    0    0 ]
        M(0, 1) += v(2);  M(0, 2) -= v(1);  M(0, 4) += v(5);  M(0, 5) -= v(4);
        M(1, 0) -= v(2);  M(1, 2) += v(0);  M(1, 3) -= v(5);  M(1, 5) += v(3);
        M(2, 0) += v(1);  M(2, 1) -= v(0);  M(2, 3) += v(4);  M(2, 4) -= v(3);
        M(3, 1) += v(5);  M(3, 2) -= v(4);
        M(4, 0) -= v(5);  M(4, 2) += v(3);
        M(5, 0) += v(4);  M(5, 1) -= v(3);
      }
      else if (n % 6 == 0)
      {
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          M(o+0, o+1) += v(o+2);  M(o+0, o+2) -= v(o+1);  M(o+0, o+4) += v(o+5);  M(o+0, o+5) -= v(o+4);
          M(o+1, o+0) -= v(o+2);  M(o+1, o+2) += v(o+0);  M(o+1, o+3) -= v(o+5);  M(o+1, o+5) += v(o+3);
          M(o+2, o+0) += v(o+1);  M(o+2, o+1) -= v(o+0);  M(o+2, o+3) += v(o+4);  M(o+2, o+4) -= v(o+3);
          M(o+3, o+1) += v(o+5);  M(o+3, o+2) -= v(o+4);
          M(o+4, o+0) -= v(o+5);  M(o+4, o+2) += v(o+3);
          M(o+5, o+0) += v(o+4);  M(o+5, o+1) -= v(o+3);
        }
      }
      else
      {
        throw std::runtime_error("Invalid dimension for addSwappedForceCrossMatrixInPlace");
      }
    }

    /*!
     * Compute swapped force cross product: icrf(f) * s for a single 6D force and motion vector.
     * This is more efficient than building the full 6x6 matrix when only one column is needed.
     * Result: icrf(f) * s where icrf is the swapped force cross matrix
     */
    template <typename T>
    auto swappedForceCrossProduct(const Eigen::MatrixBase<T> &f, const Eigen::MatrixBase<T> &s)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
      SVec<typename T::Scalar> result;
      // icrf(f) * s where icrf is:
      // [  0   f2  -f1   0   f5  -f4 ]
      // [ -f2   0   f0  -f5   0   f3 ]
      // [  f1  -f0   0   f4  -f3   0 ]
      // [  0   f5  -f4   0    0    0 ]
      // [ -f5   0   f3   0    0    0 ]
      // [  f4  -f3   0   0    0    0 ]
      result(0) = f(2)*s(1) - f(1)*s(2) + f(5)*s(4) - f(4)*s(5);
      result(1) = f(0)*s(2) - f(2)*s(0) + f(3)*s(5) - f(5)*s(3);
      result(2) = f(1)*s(0) - f(0)*s(1) + f(4)*s(3) - f(3)*s(4);
      result(3) = f(5)*s(1) - f(4)*s(2);
      result(4) = f(3)*s(2) - f(5)*s(0);
      result(5) = f(4)*s(0) - f(3)*s(1);
      return result;
    }

    /*!
     * Compute swapped force cross matrix times a matrix: out += icrf(f) * M
     * Accumulates into the pre-allocated output matrix to avoid heap allocation.
     */
    template <typename Scalar>
    void addSwappedForceCrossTimesMatrix(const DVec<Scalar> &f, const DMat<Scalar> &M,
                                         DMat<Scalar> &out)
    {
      const int n = f.rows();
      const int cols = M.cols();

      if (n == 6)
      {
        for (int c = 0; c < cols; ++c)
        {
          out(0, c) += f(2)*M(1,c) - f(1)*M(2,c) + f(5)*M(4,c) - f(4)*M(5,c);
          out(1, c) += f(0)*M(2,c) - f(2)*M(0,c) + f(3)*M(5,c) - f(5)*M(3,c);
          out(2, c) += f(1)*M(0,c) - f(0)*M(1,c) + f(4)*M(3,c) - f(3)*M(4,c);
          out(3, c) += f(5)*M(1,c) - f(4)*M(2,c);
          out(4, c) += f(3)*M(2,c) - f(5)*M(0,c);
          out(5, c) += f(4)*M(0,c) - f(3)*M(1,c);
        }
      }
      else if (n % 6 == 0)
      {
        const int num_bodies = n / 6;
        for (int b = 0; b < num_bodies; ++b)
        {
          const int o = 6 * b;
          for (int c = 0; c < cols; ++c)
          {
            out(o+0, c) += f(o+2)*M(o+1,c) - f(o+1)*M(o+2,c) + f(o+5)*M(o+4,c) - f(o+4)*M(o+5,c);
            out(o+1, c) += f(o+0)*M(o+2,c) - f(o+2)*M(o+0,c) + f(o+3)*M(o+5,c) - f(o+5)*M(o+3,c);
            out(o+2, c) += f(o+1)*M(o+0,c) - f(o+0)*M(o+1,c) + f(o+4)*M(o+3,c) - f(o+3)*M(o+4,c);
            out(o+3, c) += f(o+5)*M(o+1,c) - f(o+4)*M(o+2,c);
            out(o+4, c) += f(o+3)*M(o+2,c) - f(o+5)*M(o+0,c);
            out(o+5, c) += f(o+4)*M(o+0,c) - f(o+3)*M(o+1,c);
          }
        }
      }
      else
      {
        throw std::runtime_error("Invalid dimension for addSwappedForceCrossTimesMatrix");
      }
    }

    template <typename Scalar>
    DMat<Scalar> swappedForceCrossTimesMatrix(const DVec<Scalar> &f, const DMat<Scalar> &M)
    {
      const int n = f.rows();
      const int cols = M.cols();
      DMat<Scalar> out = DMat<Scalar>::Zero(n, cols);
      addSwappedForceCrossTimesMatrix(f, M, out);
      return out;
    }

    /*!
     * Create spatial coordinate transformation from rotation and translation
     */
    template <typename T, typename T2>
    auto createSXform(const Eigen::MatrixBase<T> &R,
                      const Eigen::MatrixBase<T2> &r)
    {
      static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                    "Must have 3x3 matrix");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 matrix");
      Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
      X.template topLeftCorner<3, 3>() = R;
      X.template bottomRightCorner<3, 3>() = R;
      X.template bottomLeftCorner<3, 3>() = -R * ori::vectorToSkewMat(r);
      return X;
    }

    /*!
     * Get rotation matrix from spatial transformation
     */
    template <typename T>
    auto rotationFromSXform(const Eigen::MatrixBase<T> &X)
    {
      static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
      RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
      return R;
    }

    /*!
     * Get translation vector from spatial transformation
     */
    template <typename T>
    auto translationFromSXform(const Eigen::MatrixBase<T> &X)
    {
      static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
      RotMat<typename T::Scalar> R = rotationFromSXform(X);
      Vec3<typename T::Scalar> r =
          -ori::matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
      return r;
    }

    /*!
     * Invert a spatial transformation (much faster than matrix inverse)
     */
    template <typename T>
    auto invertSXform(const Eigen::MatrixBase<T> &X)
    {
      static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
      RotMat<typename T::Scalar> R = rotationFromSXform(X);
      Vec3<typename T::Scalar> r =
          -ori::matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
      Mat6<typename T::Scalar> Xinv = createSXform(R.transpose(), -R * r);
      return Xinv;
    }

    /*!
     * Compute joint motion subspace vector
     */
    template <typename T>
    SVec<T> jointMotionSubspace(JointType joint, ori::CoordinateAxis axis)
    {
      Vec3<T> v(0, 0, 0);
      SVec<T> phi = SVec<T>::Zero();
      if (axis == ori::CoordinateAxis::X)
        v(0) = 1;
      else if (axis == ori::CoordinateAxis::Y)
        v(1) = 1;
      else
        v(2) = 1;

      if (joint == JointType::Prismatic)
        phi.template bottomLeftCorner<3, 1>() = v;
      else if (joint == JointType::Revolute)
        phi.template topLeftCorner<3, 1>() = v;
      else
        throw std::runtime_error("Unknown motion subspace");

      return phi;
    }

    /*!
     * Compute joint transformation
     */
    template <typename T>
    Transform<T> jointXform(JointType joint, ori::CoordinateAxis axis, T q)
    {
      Transform<T> X;
      if (joint == JointType::Revolute)
      {
        X = rotation(axis, q);
      }
      else if (joint == JointType::Prismatic)
      {
        Vec3<T> v(0, 0, 0);
        if (axis == ori::CoordinateAxis::X)
          v(0) = q;
        else if (axis == ori::CoordinateAxis::Y)
          v(1) = q;
        else if (axis == ori::CoordinateAxis::Z)
          v(2) = q;
        X = Transform<T>(RotMat<T>::Identity(), v);
      }
      else
      {
        throw std::runtime_error("Unknown joint xform\n");
      }
      return X;
    }

    /*!
     * Construct the rotational inertia of a uniform density box with a given mass.
     * @param mass Mass of the box
     * @param dims Dimensions of the box
     */
    template <typename T>
    Mat3<typename T::Scalar> rotInertiaOfBox(typename T::Scalar mass,
                                             const Eigen::MatrixBase<T> &dims)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
      Mat3<typename T::Scalar> I =
          Mat3<typename T::Scalar>::Identity() * dims.norm() * dims.norm();
      for (int i = 0; i < 3; i++)
        I(i, i) -= dims(i) * dims(i);
      I = I * mass / 12;
      return I;
    }

    /*!
     * Convert from spatial velocity to linear velocity.
     * Uses spatial velocity at the given point.
     */
    template <typename T, typename T2>
    auto spatialToLinearVelocity(const Eigen::MatrixBase<T> &v,
                                 const Eigen::MatrixBase<T2> &x)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
      Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
      Vec3<typename T::Scalar> vsLin = v.template bottomLeftCorner<3, 1>();
      Vec3<typename T::Scalar> vLinear = vsLin + vsAng.cross(x);
      return vLinear;
    }

    /*!
     * Convert from spatial velocity to angular velocity.
     */
    template <typename T>
    auto spatialToAngularVelocity(const Eigen::MatrixBase<T> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
      Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
      return vsAng;
    }

    /*!
     * Compute the classical lienear accleeration of a frame given its spatial
     * acceleration and velocity
     */
    template <typename T, typename T2>
    auto spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                     const Eigen::MatrixBase<T2> &v)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");

      Vec3<typename T::Scalar> acc;
      // classical accleration = spatial linear acc + omega x v
      acc = a.template tail<3>() + v.template head<3>().cross(v.template tail<3>());
      return acc;
    }

    /*!
     * Compute the classical lienear acceleration of a frame given its spatial
     * acceleration and velocity
     */
    template <typename T, typename T2, typename T3>
    auto spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                     const Eigen::MatrixBase<T2> &v,
                                     const Eigen::MatrixBase<T3> &x)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
      static_assert(T3::ColsAtCompileTime == 1 && T3::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");

      Vec3<typename T::Scalar> alin_x = spatialToLinearVelocity(a, x);
      Vec3<typename T::Scalar> vlin_x = spatialToLinearVelocity(v, x);

      // classical accleration = spatial linear acc + omega x v
      Vec3<typename T::Scalar> acc = alin_x + v.template head<3>().cross(vlin_x);
      return acc;
    }

    /*!
     * Apply spatial transformation to a point.
     */
    template <typename T, typename T2>
    auto sXFormPoint(const Eigen::MatrixBase<T> &X,
                     const Eigen::MatrixBase<T2> &p)
    {
      static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 vector");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");

      Mat3<typename T::Scalar> R = rotationFromSXform(X);
      Vec3<typename T::Scalar> r = translationFromSXform(X);
      Vec3<typename T::Scalar> Xp = R * (p - r);
      return Xp;
    }

    /*!
     * Convert a force at a point to a spatial force
     * @param f : force
     * @param p : point
     */
    template <typename T, typename T2>
    auto forceToSpatialForce(const Eigen::MatrixBase<T> &f,
                             const Eigen::MatrixBase<T2> &p)
    {
      static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
      SVec<typename T::Scalar> fs;
      fs.template topLeftCorner<3, 1>() = p.cross(f);
      fs.template bottomLeftCorner<3, 1>() = f;
      return fs;
    }

    /*!
     * Multiply block-diagonal inertia (stored as vector<Mat6>) by a spatial velocity vector.
     * Equivalent to assembling the block-diagonal DMat and multiplying, but avoids the allocation.
     */
    template <typename Scalar>
    DVec<Scalar> blockDiagonalTimesVector(
        const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I,
        const DVec<Scalar> &v)
    {
      const int n = (int)I.size();
      DVec<Scalar> out(6 * n);
      for (int i = 0; i < n; i++)
        out.template segment<6>(6 * i).noalias() = I[i] * v.template segment<6>(6 * i);
      return out;
    }

    /*!
     * Convert block-diagonal inertia (stored as vector<Mat6>) to a full block-diagonal DMat.
     * Only use when a dense matrix is unavoidable (e.g. initializing IA_).
     */
    template <typename Scalar>
    DMat<Scalar> blockDiagonalToMatrix(
        const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I)
    {
      const int n = (int)I.size();
      DMat<Scalar> M = DMat<Scalar>::Zero(6 * n, 6 * n);
      for (int i = 0; i < n; i++)
        M.template block<6, 6>(6 * i, 6 * i) = I[i];
      return M;
    }

    /*!
     * Compute crf(v)*I - I*crm(v) where I is stored as a vector of 6x6 blocks.
     */
    template <typename Scalar>
    void spatialInertiaCrossTerms(
        const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I_blocks,
        const DVec<Scalar> &v, DMat<Scalar> &out)
    {
      const int n = (int)I_blocks.size();
      out.setZero(6 * n, 6 * n);
      for (int b = 0; b < n; b++)
      {
        const int o = 6 * b;
        const Mat6<Scalar> &I = I_blocks[b];
        for (int r = 0; r < 6; ++r)
        {
          for (int c = 0; c < 6; ++c)
          {
            Scalar crf_part;
            switch (r) {
              case 0: crf_part = -v(o+2)*I(1,c) + v(o+1)*I(2,c) - v(o+5)*I(4,c) + v(o+4)*I(5,c); break;
              case 1: crf_part =  v(o+2)*I(0,c) - v(o+0)*I(2,c) + v(o+5)*I(3,c) - v(o+3)*I(5,c); break;
              case 2: crf_part = -v(o+1)*I(0,c) + v(o+0)*I(1,c) - v(o+4)*I(3,c) + v(o+3)*I(4,c); break;
              case 3: crf_part = -v(o+2)*I(4,c) + v(o+1)*I(5,c); break;
              case 4: crf_part =  v(o+2)*I(3,c) - v(o+0)*I(5,c); break;
              case 5: crf_part = -v(o+1)*I(3,c) + v(o+0)*I(4,c); break;
              default: crf_part = Scalar(0); break;
            }
            Scalar crm_part;
            switch (c) {
              case 0: crm_part = v(o+2)*I(r,1) - v(o+1)*I(r,2) + v(o+5)*I(r,4) - v(o+4)*I(r,5); break;
              case 1: crm_part = -v(o+2)*I(r,0) + v(o+0)*I(r,2) - v(o+5)*I(r,3) + v(o+3)*I(r,5); break;
              case 2: crm_part = v(o+1)*I(r,0) - v(o+0)*I(r,1) + v(o+4)*I(r,3) - v(o+3)*I(r,4); break;
              case 3: crm_part = v(o+2)*I(r,4) - v(o+1)*I(r,5); break;
              case 4: crm_part = -v(o+2)*I(r,3) + v(o+0)*I(r,5); break;
              case 5: crm_part = v(o+1)*I(r,3) - v(o+0)*I(r,4); break;
              default: crm_part = Scalar(0); break;
            }
            out(o+r, o+c) = crf_part - crm_part;
          }
        }
      }
    }

  } // namespace spatial

} // namespace grbda

#endif // GRBDA_SPATIAL_H
