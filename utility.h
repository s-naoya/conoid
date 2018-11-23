/**
 * @file utility.h
 * @author SAITO Naoya (saito3110.naoya@gmail.com)
 * @brief Utility definitions and fuctions and enums and classes.
 *        Based Choreonoid "EigenUtil" and "EigenTypes".
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */
#ifndef CONOID_UTILITY_H_
#define CONOID_UTILITY_H_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace con {

using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Matrix2 = Eigen::Matrix2d;
using Matrix3 = Eigen::Matrix3d;
using dmatrix = Eigen::MatrixXd;
using dvector = Eigen::VectorXd;
using Affine = Eigen::Affine3d;
using Trans = Eigen::Translation3d;
using AngleAxis = Eigen::AngleAxisd;
using Quat = Eigen::Quaterniond;

constexpr double PI = 3.14159265358979323846;
constexpr double PI_2 = 1.57079632679489661923;
constexpr double TO_DEGREE = 180.0 / PI;
constexpr double TO_RADIAN = PI / 180.0;

enum rl { null = -1, right = 0, left = 1, both = 2 };

template <typename T>
T rad2deg(T rad) {
  return static_cast<T>(TO_DEGREE * rad);
}
template <typename T>
T deg2rad(T deg) {
  return static_cast<T>(TO_RADIAN * deg);
}

inline Quat rpy2q(double roll, double pitch, double yaw) {
  return Quat(AngleAxis(yaw, Vector3::UnitZ()) *
              AngleAxis(pitch, Vector3::UnitY()) *
              AngleAxis(roll, Vector3::UnitX()));
}
inline Quat rpy2q(const Vector3 &rpy) {
  return rpy2q(rpy.x(), rpy.y(), rpy.z());
}
inline Matrix3 rpy2mat(double roll, double pitch, double yaw) {
  return rpy2q(roll, pitch, yaw).matrix();
}
inline Matrix3 rpy2mat(const Vector3 &rpy) {
  return rpy2mat(rpy.x(), rpy.y(), rpy.z());
}
inline Vector3 mat2rpy(const Matrix3 &m) { return m.eulerAngles(2, 1, 0); }
inline Vector3 q2rpy(const Quat &q) { return mat2rpy(q.toRotationMatrix()); }
inline Quat mat2q(const Matrix3 &m) { return Quat(m); }
inline rl reverse_foot(rl foot) {
  rl out(foot);
  if (out == right) { out = left; }
  else if (out == left) { out = right; }
  return out;
}

class Pose {  // 3D position and posture
 public:
  Pose() {}
  Pose(const Vector3 &translation, const Quat &rotation)
      : p_(translation), q_(rotation) {}
  Pose(const Vector3 &translation, const Matrix3 &rotation)
      : p_(translation), q_(rotation) {}
  Pose(const Affine &affine) : p_(affine.translation()), q_(affine.linear()) {}

  const Vector3 &p() const { return p_; }
  Vector3 &p() { return p_; }

  const Quat &q() const { return q_; }
  Quat &q() { return q_; }

 private:
  Vector3 p_;  // position
  Quat q_;     // rotation
};
}  // namespace con

#endif