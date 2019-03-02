#ifndef MATH_STD_MATRIX_H
#define MATH_STD_MATRIX_H

#include<math_std/common.h>

struct PV2D;
struct PV3D;
struct PV4D;

struct Matrix22
{
  Real C00, C01, C10, C11;

  Matrix22 operator+(const Matrix22 &mat) const;
  Matrix22 operator-(const Matrix22 &mat) const;
  void operator+=(const Matrix22 &mat);
  void operator-=(const Matrix22 &mat);
  Matrix22 operator*(const Real &sca) const;
  PV2D operator*(const PV2D &pv) const;
  Matrix22 operator*(const Matrix22 &mat) const;
  void operator*=(const Real &sca);
  void operator*=(const Matrix22 &mat);

  static Matrix22 identity();
};

struct Matrix33
{
  Real C00, C01, C02, C10, C11, C12, C20, C21, C22;

  Matrix33();
  Matrix33(const Real init);

  Matrix33 operator+(const Matrix33 &mat) const;
  Matrix33 operator-(const Matrix33 &mat) const;
  void operator+=(const Matrix33 &mat);
  void operator-=(const Matrix33 &mat);
  Matrix33 operator*(const Real &sca) const;
  PV3D operator*(const PV3D &pv) const;
  Matrix33 operator*(const Matrix33 &mat) const;
  void operator*=(const Real &sca);
  void operator*=(const Matrix33 &mat);
  friend std::ostream& operator<<(std::ostream &out, Matrix33 &mat);

  static Matrix33 identity();
  Real determinant();
  bool inverse(Matrix33 &matInv);
};
struct Matrix44
{
  Real C00, C01, C02, C03, C10, C11, C12, C13, C20, C21, C22, C23, C30, C31, C32, C33;

  Matrix44 operator+(const Matrix44 &mat) const;
  Matrix44 operator-(const Matrix44 &mat) const;
  void operator=(const tf::Transform &trans);
  void operator+=(const Matrix44 &mat);
  void operator-=(const Matrix44 &mat);
  Matrix44 operator*(const Real &sca) const;
  PV4D operator*(const PV4D &pv) const;
  PV3D operator*(const PV3D &pv) const;
  Matrix44 operator*(const Matrix44 &mat) const;
  void operator*=(const Real &sca);
  void operator*=(const Matrix44 &mat);
  friend std::ostream& operator<<(std::ostream &out, Matrix44 &mat);

  static Matrix44 identity();
};
struct Matrix34
{
  Real C00, C01, C02, C03, C10, C11, C12, C13, C20, C21, C22, C23;

  Matrix34();
  Matrix34(const Real value);
  Matrix34(const Matrix44 &mat);

  void operator=(const Matrix44 &mat);
  Matrix34 operator+(const Matrix34 &mat) const;
  Matrix34 operator-(const Matrix34 &mat) const;
  void operator+=(const Matrix34 &mat);
  void operator-=(const Matrix34 &mat);
  Matrix34 operator*(const Real &sca) const;
  PV3D operator*(const PV4D &pv) const;
  void operator*=(const Real &sca);
  friend std::ostream& operator<<(std::ostream &out, Matrix34 &mat);

  void set(const Real value);
};

#endif //MATH_STD_MATRIX_H
