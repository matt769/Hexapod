#include "transformations.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <stdint.h>
#include <cmath>
#include <iostream>
#include <string>

using namespace Transformations;

void printMatrix(std::string name, const Transform& M);
void printMatrix(std::string name, const tf2::Transform& tf);
void printVector(std::string name, const Vector3& v);

int main() {
  Transform A;
  printMatrix("A", A);
  Transform B = A.inverse();
  printMatrix("B", B);
  Transform C = A * B;
  printMatrix("C", C);

  Transform D, E;
  D.R_(0, 0) = 0;
  D.R_(1, 1) = 0;
  D.R_(1, 0) = -1;
  D.R_(0, 1) = 1;
  printMatrix("D", D);

  E.t_(0) = 1;
  E.t_(1) = 2;
  E.t_(2) = 3;
  printMatrix("E", E);

  Transform F = D * E;
  printMatrix("F", F);

  Transform G = F.inverse();
  printMatrix("G", G);

  Transform H = G.inverse();
  printMatrix("H", H);

  Transform J, K, L;
  float r = 0.5;
  float p = 1.0;
  float y = -0.7;
  J.R_(0, 0) = cos(r);
  J.R_(0, 1) = -sin(r);
  J.R_(1, 0) = sin(r);
  J.R_(1, 1) = cos(r);

  K.R_(0, 0) = cos(p);
  K.R_(0, 2) = sin(p);
  K.R_(2, 0) = -sin(p);
  K.R_(2, 2) = cos(p);

  L.R_(1, 1) = cos(y);
  L.R_(1, 2) = -sin(y);
  L.R_(2, 1) = sin(y);
  L.R_(2, 2) = cos(y);

  L.t_(0) = 1;
  L.t_(1) = 2;
  L.t_(2) = 3;

  Transform M = J * K * L;
  printMatrix("M", M);
  Transform Minv = M.inverse();
  printMatrix("Minv", Minv);
  Transform N = M * Minv;
  printMatrix("N", N);

  float roll = 0.5;
  float pitch = -0.9;
  float yaw = 1.3;

  Transform O, P, Q, R;
  O.R_.setRPYExtr(roll, 0, 0);
  printMatrix("O", O);
  P.R_.setRPYExtr(0, pitch, 0);
  printMatrix("P", P);
  Q.R_.setRPYExtr(0, 0, yaw);
  printMatrix("Q", Q);
  R.R_.setRPYExtr(roll, pitch, yaw);

  printMatrix("R", R);

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf2::Transform tf;
  tf.setRotation(q);

  printMatrix("tf", tf);

  Transform S = P * R * Q;
  printMatrix("S", S);
  Transform T = P.inverse() * R;
  printMatrix("T", T);
  Transform U = R * P.inverse();
  printMatrix("U", U);

  Vector3 a(2.0, 3.0, 4.0);
  printVector("a", a);
  Vector3 b = M * a;
  printVector("b", b);
  Vector3 c = M.R_ * a;
  printVector("c", c);

  std::cout << a.norm() << '\t' << b.norm() << '\t' << c.norm() << '\n';
}

void printMatrix(std::string name, const Transform& M) {
  std::cout << name << ":\n";
  for (uint8_t rowIdx = 0; rowIdx < 3; rowIdx++) {
    for (uint8_t colIdx = 0; colIdx < 3; colIdx++) {
      std::cout << M.R_(rowIdx, colIdx) << '\t';
    }
    std::cout << M.t_(rowIdx) << '\n';
  }
  std::cout << "0\t0\t0\t1\n\n";
}

void printMatrix(std::string name, const tf2::Transform& tf) {
  std::cout << name << ":\n";
  tf2::Vector3 t = tf.getOrigin();
  tf2::Matrix3x3 R = tf.getBasis();
  for (uint8_t rowIdx = 0; rowIdx < 3; rowIdx++) {
    tf2::Vector3 r = R.getRow(rowIdx);
    for (uint8_t colIdx = 0; colIdx < 3; colIdx++) {
      std::cout << r[colIdx] << '\t';
    }
    std::cout << t[rowIdx] << '\n';
  }
  std::cout << "0.0\t0.0\t0.0\t1.0\n\n";
}

void printVector(std::string name, const Vector3& v) {
  std::cout << name << ":\n";
  for (uint8_t rowIdx = 0; rowIdx < 3; rowIdx++) {
    std::cout << v(rowIdx) << '\t';
  }
  std::cout << '\n';
}