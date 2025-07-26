#ifndef EIGEN_TYPES_HPP_
#define EIGEN_TYPES_HPP_

#include <eigen3/Eigen/Dense>

/* Vector */
template <typename T>
using Vec2 = Eigen::Matrix<T, 2, 1>;

template <typename T>
using VecX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

/* Matrix */
template <typename T>
using Mat2 = Eigen::Matrix<T, 2, 2>;

#endif  // EIGEN_TYPES_HPP_