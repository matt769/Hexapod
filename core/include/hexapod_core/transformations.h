#ifndef HEXAPOD_TRANSFORMATIONS_H
#define HEXAPOD_TRANSFORMATIONS_H

#ifdef __AVR__
#include <Arduino.h>
#else
#include <stdint.h>
#endif

namespace hexapod {

class Vector3 {
 private:
  float data_[3] = {0.0f, 0.0f, 0.0f};

 public:
  Vector3();
  Vector3(float x, float y, float z);
  const float& operator()(uint8_t idx) const;
  float& operator()(uint8_t idx);
  float norm() const;
  Vector3 unit() const;
  const float& x() const;
  const float& y() const;
  const float& z() const;
  float& x();
  float& y();
  float& z();
};

class RotationMatrix {
 private:
  float data_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

 public:
  const float& operator()(uint8_t rowIdx, uint8_t colIdx) const;
  float& operator()(uint8_t rowIdx, uint8_t colIdx);
  RotationMatrix inverse() const;
  void setRPYExtr(float roll, float pitch, float yaw);  // roll -> pitch -> yaw around fixed axes
  void setRPYIntr(float roll, float pitch, float yaw);  // roll -> pitch -> yaw updating axes
};

class Quaternion {
 private:
  float data_[4] = {1.0f, 0.0f, 0.0f, 0.0f};

 public:
  Quaternion();
  Quaternion(float w, float x, float y, float z);
  Quaternion(Vector3 v);
  Quaternion conjugate() const;
  const float& w() const;
  const float& x() const;
  const float& y() const;
  const float& z() const;
  Vector3 vector() const;
  Vector3 rotate(const Vector3& v) const;
  float norm() const;
  Quaternion normalise();
  void setRPYExtr(float roll, float pitch, float yaw);
};

class Transform {
 public:
  RotationMatrix R_;
  Vector3 t_;
  Transform inverse() const;
};

Vector3 operator+(const Vector3& v1, const Vector3& v2);
Vector3 operator-(const Vector3& v);
Vector3 operator-(const Vector3& v1, const Vector3& v2);
RotationMatrix getRoll(float angle);
RotationMatrix getPitch(float angle);
RotationMatrix getYaw(float angle);
RotationMatrix operator*(const RotationMatrix& a, const RotationMatrix& b);
Vector3 operator*(const RotationMatrix& R, const Vector3& v);
Quaternion operator*(const Quaternion& a, const Quaternion& b);
Vector3 operator*(const Quaternion& q, const Vector3& v);
Transform operator*(const Transform& a, const Transform& b);
Vector3 operator*(const Transform& T, const Vector3& v);
Vector3 operator*(float scalar, const Vector3& v);
bool operator==(const Vector3& v1, const Vector3& v2);
bool operator!=(const Vector3& v1, const Vector3& v2);
RotationMatrix QuaternionToRotationMatrix(const Quaternion& q);

} // namespace hexapod

#endif // HEXAPOD_TRANSFORMATIONS_H
