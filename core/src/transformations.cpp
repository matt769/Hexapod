#include "transformations.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <stdint.h>
#include <cmath>
#endif

namespace hexapod {

static constexpr float tolerance = 0.000001f;

Vector3::Vector3() {}

Vector3::Vector3(float x, float y, float z) {
  data_[0] = x;
  data_[1] = y;
  data_[2] = z;
}

const float& Vector3::operator()(const uint8_t idx) const { return data_[idx]; }

float& Vector3::operator()(const uint8_t idx) { return data_[idx]; }

const float& Vector3::x() const { return data_[0]; }

const float& Vector3::y() const { return data_[1]; }

const float& Vector3::z() const { return data_[2]; }

float& Vector3::x() { return data_[0]; }

float& Vector3::y() { return data_[1]; }

float& Vector3::z() { return data_[2]; }

Vector3 operator+(const Vector3& v1, const Vector3& v2) {
  return Vector3(v1(0) + v2(0), v1(1) + v2(1), v1(2) + v2(2));
}

Vector3 operator-(const Vector3& v) { return Vector3(-v(0), -v(1), -v(2)); }

Vector3 operator-(const Vector3& v1, const Vector3& v2) {
  return Vector3(v1(0) - v2(0), v1(1) - v2(1), v1(2) - v2(2));
}

float Vector3::norm() const {
  return sqrtf(data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2]);
}

Vector3 Vector3::unit() const {
  Vector3 result;
  float n = norm();
  result(0) = (*this)(0) / n;
  result(1) = (*this)(1) / n;
  result(2) = (*this)(2) / n;
  return result;
}

const float& RotationMatrix::operator()(const uint8_t rowIdx, const uint8_t colIdx) const {
  return data_[rowIdx][colIdx];
}

float& RotationMatrix::operator()(const uint8_t rowIdx, const uint8_t colIdx) {
  return data_[rowIdx][colIdx];
}

RotationMatrix RotationMatrix::inverse() const {
  RotationMatrix result;
  result(0, 0) = (*this)(0, 0);
  result(0, 1) = (*this)(1, 0);
  result(0, 2) = (*this)(2, 0);
  result(0, 0) = (*this)(0, 0);
  result(1, 0) = (*this)(0, 1);
  result(1, 2) = (*this)(2, 1);
  result(1, 1) = (*this)(1, 1);
  result(2, 0) = (*this)(0, 2);
  result(2, 1) = (*this)(1, 2);
  result(2, 2) = (*this)(2, 2);
  return result;
}

// new_rotation = roll * pitch * yaw;  // RHS intrinsic rotation
void RotationMatrix::setRPYIntr(const float roll, const float pitch, const float yaw) {
  if (roll == 0 && pitch == 0 && yaw == 0) {
    return;
  }
  RotationMatrix new_rotation;
  if (roll != 0) {
    new_rotation = new_rotation * getRoll(roll);
  }
  if (pitch != 0) {
    new_rotation = new_rotation * getPitch(pitch);
  }
  if (yaw != 0) {
    new_rotation = new_rotation * getYaw(yaw);
  }
  (*this) = new_rotation;
}

// new_rotation = yaw * pitch * roll;  // LHS extrinsic rotation
void RotationMatrix::setRPYExtr(const float roll, const float pitch, const float yaw) {
  if (roll == 0 && pitch == 0 && yaw == 0) {
    return;
  }
  RotationMatrix new_rotation;
  if (yaw != 0) {
    new_rotation = new_rotation * getYaw(yaw);
  }
  if (pitch != 0) {
    new_rotation = new_rotation * getPitch(pitch);
  }
  if (roll != 0) {
    new_rotation = new_rotation * getRoll(roll);
  }
  (*this) = new_rotation;
}

Transform Transform::inverse() const {
  Transform result;
  result.R_ = R_.inverse();
  result.t_ = result.R_ * -t_;
  return result;
}

RotationMatrix getRoll(const float angle) {
  RotationMatrix result;
  result(1, 1) = cos(angle);
  result(1, 2) = -sin(angle);
  result(2, 1) = sin(angle);
  result(2, 2) = cos(angle);
  return result;
}

RotationMatrix getPitch(const float angle) {
  RotationMatrix result;
  result(0, 0) = cos(angle);
  result(0, 2) = sin(angle);
  result(2, 0) = -sin(angle);
  result(2, 2) = cos(angle);
  return result;
}

RotationMatrix getYaw(const float angle) {
  RotationMatrix result;
  result(0, 0) = cos(angle);
  result(0, 1) = -sin(angle);
  result(1, 0) = sin(angle);
  result(1, 1) = cos(angle);
  return result;
}

RotationMatrix operator*(const RotationMatrix& a, const RotationMatrix& b) {
  RotationMatrix result;
  result(0, 0) = a(0, 0) * b(0, 0) + a(0, 1) * b(1, 0) + a(0, 2) * b(2, 0);
  result(0, 1) = a(0, 0) * b(0, 1) + a(0, 1) * b(1, 1) + a(0, 2) * b(2, 1);
  result(0, 2) = a(0, 0) * b(0, 2) + a(0, 1) * b(1, 2) + a(0, 2) * b(2, 2);
  result(1, 0) = a(1, 0) * b(0, 0) + a(1, 1) * b(1, 0) + a(1, 2) * b(2, 0);
  result(1, 1) = a(1, 0) * b(0, 1) + a(1, 1) * b(1, 1) + a(1, 2) * b(2, 1);
  result(1, 2) = a(1, 0) * b(0, 2) + a(1, 1) * b(1, 2) + a(1, 2) * b(2, 2);
  result(2, 0) = a(2, 0) * b(0, 0) + a(2, 1) * b(1, 0) + a(2, 2) * b(2, 0);
  result(2, 1) = a(2, 0) * b(0, 1) + a(2, 1) * b(1, 1) + a(2, 2) * b(2, 1);
  result(2, 2) = a(2, 0) * b(0, 2) + a(2, 1) * b(1, 2) + a(2, 2) * b(2, 2);
  return result;
}

Quaternion::Quaternion() : Quaternion(1.0, 0.0, 0.0, 0.0) {}

Quaternion::Quaternion(float w, float x, float y, float z) {
  data_[0] = w;
  data_[1] = x;
  data_[2] = y;
  data_[3] = z;
}

Quaternion::Quaternion(Vector3 v) {
  data_[0] = 0.0f;
  data_[1] = v.x();
  data_[2] = v.y();
  data_[3] = v.z();
}

Quaternion Quaternion::conjugate() const { return Quaternion(w(), -x(), -y(), -z()); }

Vector3 Quaternion::vector() const { return Vector3{x(), y(), z()}; }

const float& Quaternion::w() const { return data_[0]; }

const float& Quaternion::x() const { return data_[1]; }

const float& Quaternion::y() const { return data_[2]; }

const float& Quaternion::z() const { return data_[3]; }

Vector3 Quaternion::rotate(const Vector3& v) const {
  Quaternion vq(v);
  return ((*this) * vq * (*this).conjugate()).vector();
}

float Quaternion::norm() const { return sqrt(w() * w() + x() * x() + y() * y() + z() * z()); }

Quaternion Quaternion::normalise() {
  float n = norm();
  return Quaternion{w() / n, x() / n, y() / n, z() / n};
}

void Quaternion::setRPYExtr(float roll, float pitch, float yaw) {
  if (roll == 0 && pitch == 0 && yaw == 0) {
    return;
  }
  Quaternion new_rotation;
  if (roll != 0) {
    new_rotation = new_rotation * Quaternion(cos(roll / 2.0f), sin(roll / 2.0f), 0, 0);
  }
  if (pitch != 0) {
    new_rotation = new_rotation * Quaternion(cos(pitch / 2.0f), 0, sin(pitch / 2.0f), 0);
  }
  if (yaw != 0) {
    new_rotation = new_rotation * Quaternion(cos(yaw / 2.0f), 0, 0, sin(yaw / 2.0f));
  }
  (*this) = new_rotation;
}

Quaternion operator*(const Quaternion& a, const Quaternion& b) {
  float w = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
  float x = a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y();
  float y = a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x();
  float z = a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w();
  return Quaternion{w, x, y, z};
}

Vector3 operator*(const RotationMatrix& R, const Vector3& v) {
  return Vector3(R(0, 0) * v.x() + R(0, 1) * v.y() + R(0, 2) * v.z(),
                 R(1, 0) * v.x() + R(1, 1) * v.y() + R(1, 2) * v.z(),
                 R(2, 0) * v.x() + R(2, 1) * v.y() + R(2, 2) * v.z());
}

Transform operator*(const Transform& a, const Transform& b) {
  Transform result;
  result.R_ = a.R_ * b.R_;
  result.t_ = a.t_ + a.R_ * b.t_;
  return result;
}

Vector3 operator*(const Transform& T, const Vector3& v) { return T.t_ + T.R_ * v; }

Vector3 operator*(float scalar, const Vector3& v) {
  return Vector3{v.x() * scalar, v.y() * scalar, v.z() * scalar};
}

bool operator==(const Vector3& v1, const Vector3& v2) {
  return v1.x() == v2.x() && v1.y() == v2.y() && v1.z() == v2.z();
}

bool operator!=(const Vector3& v1, const Vector3& v2) { return !(v1 == v2); }

/**
 * @brief Taken from:
 * https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
 *
 * @param q
 * @return RotationMatrix
 */
RotationMatrix QuaternionToRotationMatrix(const Quaternion& q) {
  RotationMatrix R;
  float ww = q.w() * q.w();
  float xx = q.x() * q.x();
  float yy = q.y() * q.y();
  float zz = q.z() * q.z();

  R(0, 0) = (xx - yy - zz + ww);
  R(1, 1) = (-xx + yy - zz + ww);
  R(2, 2) = (-xx - yy + zz + ww);

  float xy = q.x() * q.y();
  float zw = q.z() * q.w();
  R(1, 0) = 2.0f * (xy + zw);
  R(0, 1) = 2.0f * (xy - zw);

  float xz = q.x() * q.z();
  float yw = q.y() * q.w();
  R(2, 0) = 2.0 * (xz - yw);
  R(0, 2) = 2.0 * (xz + yw);

  float yz = q.y() * q.z();
  float xw = q.x() * q.w();
  R(2, 1) = 2.0 * (yz + xw);
  R(1, 2) = 2.0 * (yz - xw);

  return R;
}

}  // namespace hexapod
