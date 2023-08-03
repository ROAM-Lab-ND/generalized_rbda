/*!
 * @file utilities.h
 * @brief Common utility functions
 */

#ifndef GRBDA_UTILITIES_H
#define GRBDA_UTILITIES_H

#include <algorithm>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>
#include "Utils/cppTypes.h"

namespace grbda
{

  /*!
   * Are two floating point values almost equal?
   * @param a : first value
   * @param b : second value
   * @param tol : equality tolerance
   */
  template <typename T>
  bool fpEqual(T a, T b, T tol)
  {
    return std::abs(a - b) <= tol;
  }

  /*!
   * Are two std::vectors equal?
   * Compares with "!=" operator
   */
  template <typename T>
  bool vectorEqual(const std::vector<T> &a, const std::vector<T> &b)
  {
    if (a.size() != b.size())
      return false;
    for (size_t i = 0; i < a.size(); i++)
    {
      if (a[i] != b[i])
        return false;
    }
    return true;
  }

  /*!
   * Finds the greatest common element between two vectors and throws an exception if none is found.
   * NOTE: This function assumes that the vectors are sorted in ascending order.
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
   * Coerce in to be between min and max
   */
  template <typename T>
  T coerce(T in, T min, T max)
  {
    if (in < min)
    {
      in = min;
    }
    if (in > max)
    {
      in = max;
    }
    return in;
  }

  /*!
   * Apply deadband
   * @param x : input
   * @param range : deadband (+/- range around 0)
   * @return result
   */
  template <typename T>
  T deadband(T x, T range)
  {
    if (x < range && x > -range)
      x = T(0);
    return x;
  }

  /*!
   * Apply deadband to eigen type
   */
  template <typename T>
  void eigenDeadband(Eigen::MatrixBase<T> &v, typename T::Scalar band)
  {
    for (size_t i = 0; i < T::RowsAtCompileTime; i++)
    {
      for (size_t j = 0; j < T::ColsAtCompileTime; j++)
      {
        v(i, j) = deadband(v(i, j), band);
      }
    }
  }

  /*!
   * Get the sign of a number
   * 1 for positive, 0 for 0, -1 for negative...
   */
  template <typename T>
  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  /*!
   * Fill an eigen type with random numbers from a random generator and uniform
   * real distribution.
   * TODO: is there a way to make this work nicely with normal distributions too?
   */
  template <typename T>
  void fillEigenWithRandom(
      Eigen::MatrixBase<T> &v, std::mt19937 &gen,
      std::uniform_real_distribution<typename T::Scalar> &dist)
  {
    for (size_t i = 0; i < T::RowsAtCompileTime; i++)
    {
      for (size_t j = 0; j < T::ColsAtCompileTime; j++)
      {
        v(i, j) = dist(gen);
      }
    }
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

  /*!
   * Generate a random number following normal distribution
   */
  template <typename T>
  T generator_gaussian_noise(T mean, T var)
  {
    static bool hasSpare = false;
    static T rand1, rand2;

    if (hasSpare)
    {
      hasSpare = false;
      return mean + sqrt(var * rand1) * sin(rand2);
    }
    hasSpare = true;

    rand1 = rand() / ((T)RAND_MAX);
    if (rand1 < 1e-100)
      rand1 = 1e-100;
    rand1 = -2 * log(rand1);
    rand2 = rand() / ((T)RAND_MAX) * M_PI * 2.;

    // printf("rand: %f, %f\n", rand1, rand2);
    return mean + sqrt(var * rand1) * cos(rand2);
  }

  /*!
   * Convert a floating point number to a string.  Is preferable over
   * std::to_string because this uses scientific notation and won't truncate
   * small/large numbers.
   */
  template <typename T>
  std::string numberToString(T number)
  {
    static_assert(std::is_floating_point<T>::value,
                  "numberToString must use a floating point type!");
    char buffer[100];
    sprintf(buffer, "%g", number);
    return std::string(buffer);
  }

  /*!
   * map value x in (inputMin, inputMax) to (outputMin, outputMax) linearly
   */
  template <typename T>
  T mapToRange(T x, T inputMin, T inputMax, T outputMin, T outputMax)
  {
    return outputMin +
           (x - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
  }

  /*!
   * Convert eigen type to std::string.
   */
  template <typename T>
  std::string eigenToString(Eigen::MatrixBase<T> &value)
  {
    std::stringstream ss;
    ss << value;
    return ss.str();
  }

  /*!
   * Convert boolean to string (true, false)
   */
  static inline std::string boolToString(bool b)
  {
    return std::string(b ? "true" : "false");
  }

  void writeStringToFile(const std::string &fileName,
                         const std::string &fileData);
  std::string getCurrentTimeAndDate();
  std::string getConfigDirectoryPath();

  /*!
   * Get the rotation matrix coincide with euler angle
   * Intrisic ZYX rotation
   */
  template <typename T>
  void EulerZYX_2_SO3(const Vec3<T> &euler_zyx, Mat3<T> &SO3)
  {
    Mat3<T> Mat3_Z, Mat3_Y, Mat3_X;
    Mat3_Z << cos(euler_zyx[0]), -sin(euler_zyx[0]), 0, sin(euler_zyx[0]),
        cos(euler_zyx[0]), 0, 0, 0, 1;
    Mat3_Y << cos(euler_zyx[1]), 0, sin(euler_zyx[1]), 0, 1, 0,
        -sin(euler_zyx[1]), 0, cos(euler_zyx[1]);
    Mat3_X << 1, 0, 0, 0, cos(euler_zyx[2]), -sin(euler_zyx[2]), 0,
        sin(euler_zyx[2]), cos(euler_zyx[2]);

    SO3 = Mat3_Z * Mat3_Y * Mat3_X;
  }

  // Smooth Changing
  /*!
   * Interpolate with cosine (sometimes called coserp)
   */
  template <typename T>
  T smooth_change(T ini, T end, T moving_duration, T curr_time)
  {
    if (curr_time > moving_duration)
    {
      return end;
    }
    return (ini +
            (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI)));
  }

  /*!
   * Derivative of smooth_change
   */
  template <typename T>
  T smooth_change_vel(T ini, T end, T moving_duration, T curr_time)
  {
    if (curr_time > moving_duration)
    {
      return 0.0;
    }
    return ((end - ini) * 0.5 * (M_PI / moving_duration) *
            sin(curr_time / moving_duration * M_PI));
  }

  /*!
   * Derivative of smooth_change_vel
   */
  template <typename T>
  T smooth_change_acc(T ini, T end, T moving_duration, T curr_time)
  {
    if (curr_time > moving_duration)
    {
      return 0.0;
    }
    return ((end - ini) * 0.5 * (M_PI / moving_duration) *
            (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI));
  }

  /*!
   * Convert from string to float or double.
   */
  template <typename T>
  T stringToNumber(const std::string &str)
  {
    static_assert(std::is_same<T, double>::value || std::is_same<T, float>::value,
                  "stringToNumber only works for double/float");

    if (std::is_same<T, double>::value)
    {
      return std::stod(str);
    }
    else if (std::is_same<T, float>::value)
    {
      return std::stof(str);
    }
  }

  /*!
   * Convert from string to float or double
   */
  template <typename T>
  T stringToNumber(const char *str)
  {
    return stringToNumber<T>(std::string(str));
  }

  /*!
   * Convert from string to Vec3.
   */
  template <typename T>
  Vec3<T> stringToVec3(const std::string &str)
  {
    Vec3<T> v;
    size_t i = 0;

    // seek past whitespace
    while (str.at(i) == ' ')
      i++;

    if (str.at(i) == '[')
    {
      i++;
    }
    else
    {
      throw std::runtime_error("stringToVec3 didn't find open bracket");
    }

    // seek past whitespace
    while (str.at(i) == ' ')
      i++;
    size_t start = i;

    // seek to end of first number
    while (str.at(i) != ',')
      i++;
    v[0] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ',')
      i++;
    v[1] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ']')
      i++;
    v[2] = stringToNumber<T>(str.substr(start, i - start));
    return v;
  }

  template <typename T>
  Vec6<T> stringToVec6(const std::string &str)
  {
    Vec6<T> v;
    size_t i = 0;

    // seek past whitespace
    while (str.at(i) == ' ')
      i++;

    if (str.at(i) == '[')
    {
      i++;
    }
    else
    {
      throw std::runtime_error("stringToVec3 didn't find open bracket");
    }

    // seek past whitespace
    while (str.at(i) == ' ')
      i++;
    size_t start = i;

    // seek to end of first number
    while (str.at(i) != ',')
      i++;
    v[0] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ',')
      i++;
    v[1] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ',')
      i++;
    v[2] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ',')
      i++;
    v[3] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ',')
      i++;
    v[4] = stringToNumber<T>(str.substr(start, i - start));
    i++;

    while (str.at(i) == ' ')
      i++;
    start = i;
    while (str.at(i) != ']')
      i++;
    v[5] = stringToNumber<T>(str.substr(start, i - start));
    return v;
  }

  std::string getLcmUrl(s64 ttl);

} // namespace grbda

#endif // PROJECT_UTILITIES_H
