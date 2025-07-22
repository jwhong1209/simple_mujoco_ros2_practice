/** @file  eigen_types.hpp
 *  @brief Common Eigen types that are only valid in C++
 *  @copyright This file is referred from Cheetah-software by MIT Biomimetics Lab
 */

#ifndef EIGEN_TYPES_HPP_
#define EIGEN_TYPES_HPP_

#include <Eigen/Dense>

/* Orientation */
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;  // Rotation Matrix

// ! - the order of Eigen::Quaternion is: x * i + y * j + z * k + w
// ! - while MJCF data is: w + x * i + y * j + z * k
template <typename T>
using Quat = typename Eigen::Quaternion<T>;  // 4x1 Vector for Quaternion

/* Vector*/
template <typename T>
using VecX = Eigen::Matrix<T, Eigen::Dynamic, 1>;  // Dynamic vector

template <typename T>
using Vec2 = Eigen::Matrix<T, 2, 1>;  // 2x1 Vector

template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;  // 3x1 Vector

template <typename T>
using Vec4 = Eigen::Matrix<T, 4, 1>;  // 4x1 Vector

template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;  // 6x1 Vector

template <typename T>
using Vec7 = Eigen::Matrix<T, 7, 1>;  // 7x1 Vector

/* Matrix */
template <typename T>
using MatX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;  // Dynamic matrix

template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;  // 3x3 Matrix

template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;  // 4x4 Matrix

template <typename T>
using Mat6 = Eigen::Matrix<T, 6, 6>;  // 6x6 Matrix

template <typename T>
using Mat7 = Eigen::Matrix<T, 7, 7>;  // 7x7 Matrix

template <typename T>
using Mat67 = Eigen::Matrix<T, 6, 7>;  // 6x7 Matrix for Jacobian

template <typename T>
using Mat76 = Eigen::Matrix<T, 7, 6>;  // 7x6 Matrix for Jacobian

#endif  // EIGEN_TYPES_HPP_