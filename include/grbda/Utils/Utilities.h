/*!
 * @file Utilities.h
 * @brief Common utility functions
 */

#ifndef GRBDA_UTILITIES_H
#define GRBDA_UTILITIES_H

#include <random>
#include "cppTypes.h"

namespace grbda
{
  /*!
   * Square a number
   */
  template <typename T>
  T square(T a)
  {
    return a * a;
  }

  /*!
   * Square root of a number
   */
  inline double sqrt(double a)
  {
    return std::sqrt(a);
  }

  inline casadi::SX sqrt(casadi::SX a)
  {
    return casadi::SX::sqrt(a);
  }

  /*!
   * Generate a random number
   */
  template <typename T>
  T random()
  {
    return T(std::rand()) / T(RAND_MAX);
  }

  /*!
   * Generate a std::vector of random numbers
   */
  template <typename T>
  std::vector<T> random(int n)
  {
    std::vector<T> v(n);
    for (int i = 0; i < n; ++i)
    {
      v[i] = random<T>();
    }
    return v;
  }

  inline bool randomBool()
  {
    static auto gen = std::bind(std::uniform_int_distribution<>(0, 1),
                                std::default_random_engine());
    return gen();
  }

  /*!
   * Generate a std::vector of zeros
   */
  template <typename T>
  std::vector<T> zeros(int n)
  {
    std::vector<T> v(n);
    for (int i = 0; i < n; ++i)
    {
      v[i] = T(0);
    }
    return v;
  }

  /*!
   * Convert radians to degrees
   */
  template <typename T>
  T rad2deg(T rad)
  {
    return rad * T(180) / T(M_PI);
  }

  /*!
   * Convert degrees to radians
   */
  template <typename T>
  T deg2rad(T deg)
  {
    return deg * T(M_PI) / T(180);
  }

  /*!
   * Check if an Eigen matrix is positive definite
   */
  template <typename T>
  bool isPositiveDefinite(const Eigen::MatrixBase<T> &A)
  {
    // Symmetry check
    if (!A.isApprox(A.transpose()))
    {
      return false;
    }

    // Positive eignvalue check
    Eigen::LLT<T> llt_of_A(A);
    if (llt_of_A.info() == Eigen::NumericalIssue)
    {
      return false;
    }

    return true;
  }

  /*!
   * Check if an Eigen vector is approximately zero
   * NOTE: Operates on the entire vector, not element-wise
   */
  template <typename T>
  bool nearZero(const Eigen::MatrixBase<T> &vec, const typename T::Scalar &tol = 1e-8)
  {
    if (vec.norm() < tol)
      return true;
    return false;
  }

  /*!
   * These variants of nearZero are used to handle the casadi::SX type because casadi::SX type 
   * objects cannot be converted to bool
   */
  template <typename T>
  bool nearZeroDefaultTrue(const Eigen::MatrixBase<T> &vec, const typename T::Scalar &tol = 1e-8)
  {
    return nearZero(vec, tol);
  }

  template <>
  inline bool nearZeroDefaultTrue<DVec<casadi::SX>>(const Eigen::MatrixBase<DVec<casadi::SX>> &vec,
                                                    const casadi::SX &tol)
  {
    return true;
  }

  template <typename T>
  bool nearZeroDefaultFalse(const Eigen::MatrixBase<T> &vec, const typename T::Scalar &tol = 1e-8)
  {
    return nearZero(vec, tol);
  }

  template <>
  inline bool nearZeroDefaultFalse<DVec<casadi::SX>>(const Eigen::MatrixBase<DVec<casadi::SX>> &vec,
                                                     const casadi::SX &tol)
  {
    return false;
  }

  /*!
   * Overloaded functions for taking the inverse of a Eigen matrix
   * - Standard Eigen matrices are inverted using the .inverse() function
   * - Casadi matrices are symbolically inverted using the casadi::SX::inv() function
   */
  template <typename T>
  DMat<typename T::Scalar> matrixInverse(const Eigen::MatrixBase<T> &mat)
  {
    return mat.inverse();
  }

  template <>
  inline DMat<casadi::SX>
  matrixInverse<DMat<casadi::SX>>(const Eigen::MatrixBase<DMat<casadi::SX>> &mat)
  {
    casadi::SX cs_mat(mat.rows(), mat.cols());
    casadi::copy(mat, cs_mat);

    casadi::SX cs_mat_inv = casadi::SX::inv(cs_mat);

    DMat<casadi::SX> mat_out(mat.rows(), mat.cols());
    casadi::copy(cs_mat_inv, mat_out);

    return mat_out;
  }

  /*!
   * Left pseudo inverse of an Eigen matrix.
   * Applicable when mat has linearly independent columns
   */
  template <typename T>
  DMat<typename T::Scalar> matrixLeftPseudoInverse(const Eigen::MatrixBase<T> &mat)
  {
    return mat.completeOrthogonalDecomposition().pseudoInverse();
  }

  template <>
  inline DMat<casadi::SX>
  matrixLeftPseudoInverse<DMat<casadi::SX>>(const Eigen::MatrixBase<DMat<casadi::SX>> &mat)
  {
    const DMat<casadi::SX> tmp = mat.transpose() * mat;
    return matrixInverse(tmp) * mat.transpose();
  }

  /*!
   * Right pseudo inverse of an Eigen matrix.
   * Applicable when mat has linearly independent rows
   */
  template <typename T>
  DMat<typename T::Scalar> matrixRightPseudoInverse(const Eigen::MatrixBase<T> &mat)
  {
    return mat.completeOrthogonalDecomposition().pseudoInverse();
  }

  template <>
  inline DMat<casadi::SX>
  matrixRightPseudoInverse<DMat<casadi::SX>>(const Eigen::MatrixBase<DMat<casadi::SX>> &mat)
  {
    const DMat<casadi::SX> tmp = mat * mat.transpose();
    return mat.transpose() * matrixInverse(tmp);
  }

  /*!
   * Solve a linear system of Eigen matrices
   */
  template <typename T>
  DVec<typename T::Scalar> solveLinearSystem(const Eigen::MatrixBase<T> &A,
                                             const DVec<typename T::Scalar> &b)
  {
    return A.colPivHouseholderQr().solve(b);
  }

  template <>
  inline DVec<casadi::SX> solveLinearSystem(const Eigen::MatrixBase<DMat<casadi::SX>> &A,
                                            const DVec<casadi::SX> &b)
  {
    casadi::SX cs_A(A.rows(), A.cols());
    casadi::copy(A, cs_A);

    casadi::SX cs_b(b.rows(), b.cols());
    casadi::copy(b, cs_b);

    casadi::SX cs_x = casadi::SX::solve(cs_A, cs_b);

    DVec<casadi::SX> x(cs_x.size1());
    casadi::copy(cs_x, x);

    return x;
  }

  /*!
   * Analaogous to Eigen::LLT, but for casadi::SX matrices
   */
  class CasadiLLT
  {
  public:
    CasadiLLT() {}
    CasadiLLT(const DMat<casadi::SX> &mat)
    {
      Ainv_.resize(mat.rows(), mat.cols());

      casadi::SX cs_mat(mat.rows(), mat.cols());
      casadi::copy(mat, cs_mat);

      casadi::SX cs_mat_inv = casadi::SX::inv(cs_mat);

      DMat<casadi::SX> mat_out(mat.rows(), mat.cols());
      casadi::copy(cs_mat_inv, Ainv_);
    }

    template <typename Derived>
    DMat<casadi::SX> solve(const Eigen::MatrixBase<Derived> &b) const
    {
      return Ainv_ * b;
    }

  private:
    DMat<casadi::SX> Ainv_;
  };

  template <typename Scalar>
  struct CorrectMatrixLltType
  {
    using type = Eigen::LLT<DMat<Scalar>>;
  };

  // Specialization for casadi::SX
  template <>
  struct CorrectMatrixLltType<casadi::SX>
  {
    using type = CasadiLLT;
  };

  /*!
   * Analaogous to Eigen::ColPivHouseholderQR, but for casadi::SX matrices
   */
  class CasadiInverse
  {
  public:
    CasadiInverse() {}
    CasadiInverse(const DMat<casadi::SX> &mat)
    {
      Ainv_.resize(mat.rows(), mat.cols());

      casadi::SX cs_mat(mat.rows(), mat.cols());
      casadi::copy(mat, cs_mat);

      casadi::SX cs_mat_inv = casadi::SX::inv(cs_mat);

      DMat<casadi::SX> mat_out(mat.rows(), mat.cols());
      casadi::copy(cs_mat_inv, Ainv_);
    }

    template <typename Derived>
    DMat<casadi::SX> solve(const Eigen::MatrixBase<Derived> &b) const
    {
      return Ainv_ * b;
    }

  private:
    DMat<casadi::SX> Ainv_;
  };

  template <typename Scalar>
  struct CorrectMatrixInverseType
  {
    using type = Eigen::ColPivHouseholderQR<DMat<Scalar>>;
  };

  // Specialization for casadi::SX
  template <>
  struct CorrectMatrixInverseType<casadi::SX>
  {
    using type = CasadiInverse;
  };

  /*!
   * Finds the greatest common element between two vectors and throws an exception if none is found.
   * NOTE: This function assumes that the vectors are sorted in descending order.
   */
  template <typename T>
  int greatestCommonElement(const std::vector<T> &vec1, const std::vector<T> &vec2)
  {
    // Find the largest common element between the two vectors
    for (T elem : vec1)
    {
      if (std::find(vec2.begin(), vec2.end(), elem) != vec2.end())
      {
        // Common element found, return it
        return elem;
      }
    }

    // No common element found, throw an exception
    throw std::runtime_error("No common element found.");
  }

  /*!
   * Append an eigen vector to an existing eigen vector
   */
  template <typename T>
  DVec<T> appendEigenVector(const DVec<T> &vec, const DVec<T> &appendage)
  {
    DVec<T> vec_out = DVec<T>::Zero(vec.rows() + appendage.rows());
    vec_out << vec, appendage;
    return vec_out;
  }

  /*!
   * Append an eigen matrix to an existing eigen matrix in block diagonal fashion
   */
  template <typename T>
  DMat<T> appendEigenMatrix(const DMat<T> &mat, const DMat<T> &appendage)
  {
    DMat<T> mat_out = DMat<T>::Zero(mat.rows() + appendage.rows(),
                                    mat.cols() + appendage.cols());
    mat_out << mat, DMat<T>::Zero(mat.rows(), appendage.cols()),
        DMat<T>::Zero(appendage.rows(), mat.cols()), appendage;
    return mat_out;
  }

} // namespace grbda

#endif // PROJECT_UTILITIES_H
