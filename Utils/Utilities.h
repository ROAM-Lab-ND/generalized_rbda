/*!
 * @file Utilities.h
 * @brief Common utility functions
 */

#ifndef GRBDA_UTILITIES_H
#define GRBDA_UTILITIES_H

#include "cppTypes.h"

namespace grbda
{

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
        DMat<double>::Zero(appendage.rows(), mat.cols()), appendage;
    return mat_out;
  }

} // namespace grbda

#endif // PROJECT_UTILITIES_H
