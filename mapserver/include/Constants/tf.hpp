#ifndef CONSTANTS_TF_HPP_
#define CONSTANTS_TF_HPP_

#include <math.h>
#include "common.hpp"
#include <Eigen/Dense>

namespace constants {
namespace ctf {
template<typename T>
constexpr Eigen::Matrix<T, 4, 4> trans(T x, T y, T z) {
  Eigen::Matrix<T, 4, 4> returnMatrix;
  returnMatrix << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
  return returnMatrix;
}
template<typename T>
constexpr Eigen::Matrix<T, 4, 4> rotX(T angle) {
  Eigen::Matrix<T, 4, 4> returnMatrix;
  returnMatrix << 1, 0, 0, 0, 0, cos(angle), -sin(angle), 0, 0, sin(angle), cos(
      angle), 0, 0, 0, 0, 1;
  return returnMatrix;
}
template<typename T>
constexpr Eigen::Matrix<T, 4, 4> rotY(T angle) {
  Eigen::Matrix<T, 4, 4> returnMatrix;
  returnMatrix << cos(angle), 0, sin(angle), 0, 0, 1, 0, 0, -sin(angle), 0, cos(
      angle), 0, 0, 0, 0, 1;
  return returnMatrix;
}
template<typename T>
constexpr Eigen::Matrix<T, 4, 4> rotZ(T angle) {
  Eigen::Matrix<T, 4, 4> returnMatrix;
  returnMatrix << cos(angle), -sin(angle), 0, 0, sin(angle), cos(angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  return returnMatrix;
}
template<typename T>
constexpr Eigen::Matrix<T, 3, 3> getRot(Eigen::Matrix<T, 4, 4> M) {
  Eigen::Matrix<T, 3, 3> returnMatrix;
  return M.template block<3, 3>(0, 0);
}
template<typename T>
constexpr Eigen::Matrix<T, 3, 1> getTrans(Eigen::Matrix<T, 4, 4> M) {
  return M.template block<3, 1>(0, 3);
}
}
}

#endif // CONSTANTS_TF_HPP_
