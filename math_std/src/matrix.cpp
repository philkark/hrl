#include<math_std/matrix.h>
#include<math_std/pv.h>

//############# Matrix22 #############

Matrix22 Matrix22::operator+(const Matrix22 &mat) const
{
  Matrix22 matNew;
  matNew.C00 = C00 + mat.C00;
  matNew.C01 = C01 + mat.C01;
  matNew.C10 = C10 + mat.C10;
  matNew.C11 = C11 + mat.C11;
  return matNew;
}
Matrix22 Matrix22::operator-(const Matrix22 &mat) const
{
  Matrix22 matNew;
  matNew.C00 = C00 - mat.C00;
  matNew.C01 = C01 - mat.C01;
  matNew.C10 = C10 - mat.C10;
  matNew.C11 = C11 - mat.C11;
  return matNew;
}
void Matrix22::operator+=(const Matrix22 &mat)
{
  C00 += mat.C00;
  C01 += mat.C01;
  C10 += mat.C10;
  C11 += mat.C11;
}
void Matrix22::operator-=(const Matrix22 &mat)
{
  C00 -= mat.C00;
  C01 -= mat.C01;
  C10 -= mat.C10;
  C11 -= mat.C11;
}
Matrix22 Matrix22::operator*(const Real &sca) const
{
  Matrix22 matNew;
  matNew.C00 = C00 * sca;
  matNew.C01 = C01 * sca;
  matNew.C10 = C10 * sca;
  matNew.C11 = C11 * sca;
  return matNew;
}
PV2D Matrix22::operator*(const PV2D &pv) const
{
  PV2D pvNew;
  pvNew.C0 = C00 * pv.C0 + C01 * pv.C1;
  pvNew.C1 = C10 * pv.C0 + C11 * pv.C1;
  return pvNew;
}
Matrix22 Matrix22::operator*(const Matrix22 &mat) const
{
  Matrix22 matNew;
  matNew.C00 = C00 * mat.C00 + C01 * mat.C10;
  matNew.C01 = C00 * mat.C01 + C01 * mat.C11;
  matNew.C10 = C10 * mat.C00 + C11 * mat.C10;
  matNew.C11 = C10 * mat.C01 + C11 * mat.C11;
  return matNew;
}
void Matrix22::operator*=(const Real &sca)
{
  C00 *= sca;
  C01 *= sca;
  C10 *= sca;
  C11 *= sca;
}
void Matrix22::operator*=(const Matrix22 &mat)
{
  Matrix22 matNew;
  matNew.C00 = C00 * mat.C00 + C01 * mat.C10;
  matNew.C01 = C00 * mat.C01 + C01 * mat.C11;
  matNew.C10 = C10 * mat.C00 + C11 * mat.C10;
  matNew.C11 = C10 * mat.C01 + C11 * mat.C11;
  C00 = matNew.C00;
  C01 = matNew.C01;
  C10 = matNew.C10;
  C11 = matNew.C11;
}

Matrix22 Matrix22::identity()
{
  Matrix22 mat;
  mat.C00 = mat.C11 = 1;
  mat.C01 = mat.C10 = 0;
  return mat;
}

//############# Matrix33 #############

Matrix33::Matrix33()
{

}
Matrix33::Matrix33(const Real init) :
    C00(init), C01(init), C02(init), C10(init), C11(init), C12(init), C20(init), C21(init), C22(init)
{

}

Matrix33 Matrix33::operator+(const Matrix33 &mat) const
{
  Matrix33 matNew;
  matNew.C00 = C00 + mat.C00;
  matNew.C01 = C01 + mat.C01;
  matNew.C02 = C02 + mat.C02;
  matNew.C10 = C10 + mat.C10;
  matNew.C11 = C11 + mat.C11;
  matNew.C12 = C12 + mat.C12;
  matNew.C20 = C20 + mat.C20;
  matNew.C21 = C21 + mat.C21;
  matNew.C22 = C22 + mat.C22;
  return matNew;
}
Matrix33 Matrix33::operator-(const Matrix33 &mat) const
{
  Matrix33 matNew;
  matNew.C00 = C00 - mat.C00;
  matNew.C01 = C01 - mat.C01;
  matNew.C02 = C02 - mat.C02;
  matNew.C10 = C10 - mat.C10;
  matNew.C11 = C11 - mat.C11;
  matNew.C12 = C12 - mat.C12;
  matNew.C20 = C20 - mat.C20;
  matNew.C21 = C21 - mat.C21;
  matNew.C22 = C22 - mat.C22;
  return matNew;
}
void Matrix33::operator+=(const Matrix33 &mat)
{
  C00 += mat.C00;
  C01 += mat.C01;
  C02 += mat.C02;
  C10 += mat.C10;
  C11 += mat.C11;
  C12 += mat.C12;
  C20 += mat.C20;
  C21 += mat.C21;
  C22 += mat.C22;
}
void Matrix33::operator-=(const Matrix33 &mat)
{
  C00 -= mat.C00;
  C01 -= mat.C01;
  C02 -= mat.C02;
  C10 -= mat.C10;
  C11 -= mat.C11;
  C12 -= mat.C12;
  C20 -= mat.C20;
  C21 -= mat.C21;
  C22 -= mat.C22;
}
Matrix33 Matrix33::operator*(const Real &sca) const
{
  Matrix33 matNew;
  matNew.C00 = C00 * sca;
  matNew.C01 = C01 * sca;
  matNew.C02 = C02 * sca;
  matNew.C10 = C10 * sca;
  matNew.C11 = C11 * sca;
  matNew.C12 = C12 * sca;
  matNew.C20 = C20 * sca;
  matNew.C21 = C21 * sca;
  matNew.C22 = C22 * sca;
  return matNew;
}
PV3D Matrix33::operator*(const PV3D &pv) const
{
  PV3D pvNew;
  pvNew.C0 = C00 * pv.C0 + C01 * pv.C1 + C02 * pv.C2;
  pvNew.C1 = C10 * pv.C0 + C11 * pv.C1 + C12 * pv.C2;
  pvNew.C2 = C20 * pv.C0 + C21 * pv.C1 + C22 * pv.C2;
  return pvNew;
}
Matrix33 Matrix33::operator*(const Matrix33 &mat) const
{
  Matrix33 matNew;
  matNew.C00 = C00 * mat.C00 + C01 * mat.C10 + C02 * mat.C20;
  matNew.C01 = C00 * mat.C01 + C01 * mat.C11 + C02 * mat.C21;
  matNew.C02 = C00 * mat.C02 + C01 * mat.C12 + C02 * mat.C22;
  matNew.C10 = C10 * mat.C00 + C11 * mat.C10 + C12 * mat.C20;
  matNew.C11 = C10 * mat.C01 + C11 * mat.C11 + C12 * mat.C21;
  matNew.C12 = C10 * mat.C02 + C11 * mat.C12 + C12 * mat.C22;
  matNew.C20 = C20 * mat.C00 + C21 * mat.C10 + C22 * mat.C20;
  matNew.C21 = C20 * mat.C01 + C21 * mat.C11 + C22 * mat.C21;
  matNew.C22 = C20 * mat.C02 + C21 * mat.C12 + C22 * mat.C22;
  return matNew;
}
void Matrix33::operator*=(const Real &sca)
{
  C00 *= sca;
  C01 *= sca;
  C02 *= sca;
  C10 *= sca;
  C11 *= sca;
  C12 *= sca;
  C20 *= sca;
  C21 *= sca;
  C22 *= sca;
}
void Matrix33::operator*=(const Matrix33 &mat)
{
  Matrix33 matNew;
  matNew.C00 = C00 * mat.C00 + C01 * mat.C10 + C02 * mat.C20;
  matNew.C01 = C00 * mat.C01 + C01 * mat.C11 + C02 * mat.C21;
  matNew.C02 = C00 * mat.C02 + C01 * mat.C12 + C02 * mat.C22;
  matNew.C10 = C10 * mat.C00 + C11 * mat.C10 + C12 * mat.C20;
  matNew.C11 = C10 * mat.C01 + C11 * mat.C11 + C12 * mat.C21;
  matNew.C12 = C10 * mat.C02 + C11 * mat.C12 + C12 * mat.C22;
  matNew.C20 = C20 * mat.C00 + C21 * mat.C10 + C22 * mat.C20;
  matNew.C21 = C20 * mat.C01 + C21 * mat.C11 + C22 * mat.C21;
  matNew.C22 = C20 * mat.C02 + C21 * mat.C12 + C22 * mat.C22;
  C00 = matNew.C00;
  C01 = matNew.C01;
  C02 = matNew.C02;
  C10 = matNew.C10;
  C11 = matNew.C11;
  C12 = matNew.C12;
  C20 = matNew.C20;
  C21 = matNew.C21;
  C22 = matNew.C22;
}
std::ostream& operator<<(std::ostream &out, Matrix33 &mat)
{
  out << std::setprecision(7) << std::fixed;
  out << std::setw(10) << mat.C00 << ", " << std::setw(10) << mat.C01 << ", " << std::setw(10) << mat.C02 << std::endl;
  out << std::setw(10) << mat.C10 << ", " << std::setw(10) << mat.C11 << ", " << std::setw(10) << mat.C12 << std::endl;
  out << std::setw(10) << mat.C20 << ", " << std::setw(10) << mat.C21 << ", " << std::setw(10) << mat.C22 << std::endl;
  return out;
}

Matrix33 Matrix33::identity()
{
  Matrix33 mat;
  mat.C00 = mat.C11 = mat.C22 = 1;
  mat.C01 = mat.C02 = mat.C10 = mat.C12 = mat.C20 = mat.C21 = 0;
  return mat;
}

Real Matrix33::determinant()
{
  return C00 * (C11 * C22 - C12 * C21) + C01 * (C20 * C12 - C10 * C22) + C02 * (C10 * C21 - C11 * C20);
}
bool Matrix33::inverse(Matrix33 &matInv)
{
  const Real det = determinant();
  if (det == 0.0)
    return false;

  const Real detRecip = 1 / determinant();

  matInv.C00 = detRecip * (C11 * C22 - C12 * C21);
  matInv.C01 = detRecip * (C02 * C21 - C01 * C22);
  matInv.C02 = detRecip * (C01 * C12 - C02 * C11);
  matInv.C10 = detRecip * (C12 * C20 - C10 * C22);
  matInv.C11 = detRecip * (C00 * C22 - C02 * C20);
  matInv.C12 = detRecip * (C02 * C10 - C00 * C12);
  matInv.C20 = detRecip * (C10 * C21 - C11 * C20);
  matInv.C21 = detRecip * (C01 * C20 - C00 * C21);
  matInv.C22 = detRecip * (C00 * C11 - C01 * C10);
  return true;
}

//############# Matrix44 #############

Matrix44 Matrix44::operator+(const Matrix44 &mat) const
{
  Matrix44 matNew;
  matNew.C00 = C00 + mat.C00;
  matNew.C01 = C01 + mat.C01;
  matNew.C02 = C02 + mat.C02;
  matNew.C03 = C03 + mat.C03;
  matNew.C10 = C10 + mat.C10;
  matNew.C11 = C11 + mat.C11;
  matNew.C12 = C12 + mat.C12;
  matNew.C13 = C13 + mat.C13;
  matNew.C20 = C20 + mat.C20;
  matNew.C21 = C21 + mat.C21;
  matNew.C22 = C22 + mat.C22;
  matNew.C23 = C23 + mat.C23;
  matNew.C30 = C30 + mat.C30;
  matNew.C31 = C31 + mat.C31;
  matNew.C32 = C32 + mat.C32;
  matNew.C33 = C33 + mat.C33;
  return matNew;
}
Matrix44 Matrix44::operator-(const Matrix44 &mat) const
{
  Matrix44 matNew;
  matNew.C00 = C00 - mat.C00;
  matNew.C01 = C01 - mat.C01;
  matNew.C02 = C02 - mat.C02;
  matNew.C03 = C03 - mat.C03;
  matNew.C10 = C10 - mat.C10;
  matNew.C11 = C11 - mat.C11;
  matNew.C12 = C12 - mat.C12;
  matNew.C13 = C13 - mat.C13;
  matNew.C20 = C20 - mat.C20;
  matNew.C21 = C21 - mat.C21;
  matNew.C22 = C22 - mat.C22;
  matNew.C23 = C23 - mat.C23;
  matNew.C30 = C30 - mat.C30;
  matNew.C31 = C31 - mat.C31;
  matNew.C32 = C32 - mat.C32;
  matNew.C33 = C33 - mat.C33;
  return matNew;
}
void Matrix44::operator=(const tf::Transform &trans)
{
  const tf::Matrix3x3 &mat(trans.getBasis());
  const tf::Vector3 &vec(trans.getOrigin());
  C00 = mat[0][0];
  C01 = mat[0][1];
  C02 = mat[0][2];
  C03 = vec[0];
  C10 = mat[1][0];
  C11 = mat[1][1];
  C12 = mat[1][2];
  C13 = vec[1];
  C20 = mat[2][0];
  C21 = mat[2][1];
  C22 = mat[2][2];
  C23 = vec[2];
  C30 = C31 = C32 = 0.0;
  C33 = 1.0;
}
void Matrix44::operator+=(const Matrix44 &mat)
{
  C00 += mat.C00;
  C01 += mat.C01;
  C02 += mat.C02;
  C03 += mat.C03;
  C10 += mat.C10;
  C11 += mat.C11;
  C12 += mat.C12;
  C13 += mat.C13;
  C20 += mat.C20;
  C21 += mat.C21;
  C22 += mat.C22;
  C23 += mat.C23;
  C30 += mat.C30;
  C31 += mat.C31;
  C32 += mat.C32;
  C33 += mat.C33;
}
void Matrix44::operator-=(const Matrix44 &mat)
{
  C00 -= mat.C00;
  C01 -= mat.C01;
  C02 -= mat.C02;
  C03 -= mat.C03;
  C10 -= mat.C10;
  C11 -= mat.C11;
  C12 -= mat.C12;
  C13 -= mat.C13;
  C20 -= mat.C20;
  C21 -= mat.C21;
  C22 -= mat.C22;
  C23 -= mat.C23;
  C30 -= mat.C30;
  C31 -= mat.C31;
  C32 -= mat.C32;
  C33 -= mat.C33;
}
Matrix44 Matrix44::operator*(const Real &sca) const
{
  Matrix44 matNew;
  matNew.C00 = C00 * sca;
  matNew.C01 = C01 * sca;
  matNew.C02 = C02 * sca;
  matNew.C03 = C03 * sca;
  matNew.C10 = C10 * sca;
  matNew.C11 = C11 * sca;
  matNew.C12 = C12 * sca;
  matNew.C13 = C13 * sca;
  matNew.C20 = C20 * sca;
  matNew.C21 = C21 * sca;
  matNew.C22 = C22 * sca;
  matNew.C23 = C23 * sca;
  matNew.C30 = C30 * sca;
  matNew.C31 = C31 * sca;
  matNew.C32 = C32 * sca;
  matNew.C33 = C33 * sca;
  return matNew;
}
PV4D Matrix44::operator*(const PV4D &pv) const
{
  PV4D pvNew;
  pvNew.C0 = C00 * pv.C0 + C01 * pv.C1 + C02 * pv.C2 + C03 * pv.C3;
  pvNew.C1 = C10 * pv.C0 + C11 * pv.C1 + C12 * pv.C2 + C13 * pv.C3;
  pvNew.C2 = C20 * pv.C0 + C21 * pv.C1 + C22 * pv.C2 + C23 * pv.C3;
  pvNew.C3 = C30 * pv.C0 + C31 * pv.C1 + C32 * pv.C2 + C33 * pv.C3;
  return pvNew;
}
PV3D Matrix44::operator*(const PV3D &pv) const
{
  PV3D pvNew;
  pvNew.C0 = C00 * pv.C0 + C01 * pv.C1 + C02 * pv.C2 + C03;
  pvNew.C1 = C10 * pv.C0 + C11 * pv.C1 + C12 * pv.C2 + C13;
  pvNew.C2 = C20 * pv.C0 + C21 * pv.C1 + C22 * pv.C2 + C23;
  return pvNew;
}
Matrix44 Matrix44::operator*(const Matrix44 &mat) const
{
  Matrix44 matNew;
  matNew.C00 = C00 * mat.C00 + C01 * mat.C10 + C02 * mat.C20 + C03 * mat.C30;
  matNew.C01 = C00 * mat.C01 + C01 * mat.C11 + C02 * mat.C21 + C03 * mat.C31;
  matNew.C02 = C00 * mat.C02 + C01 * mat.C12 + C02 * mat.C22 + C03 * mat.C32;
  matNew.C03 = C00 * mat.C03 + C01 * mat.C13 + C02 * mat.C23 + C03 * mat.C33;
  matNew.C10 = C10 * mat.C00 + C11 * mat.C10 + C12 * mat.C20 + C13 * mat.C30;
  matNew.C11 = C10 * mat.C01 + C11 * mat.C11 + C12 * mat.C21 + C13 * mat.C31;
  matNew.C12 = C10 * mat.C02 + C11 * mat.C12 + C12 * mat.C22 + C13 * mat.C32;
  matNew.C13 = C10 * mat.C03 + C11 * mat.C13 + C12 * mat.C23 + C13 * mat.C33;
  matNew.C20 = C20 * mat.C00 + C21 * mat.C10 + C22 * mat.C20 + C23 * mat.C30;
  matNew.C21 = C20 * mat.C01 + C21 * mat.C11 + C22 * mat.C21 + C23 * mat.C31;
  matNew.C22 = C20 * mat.C02 + C21 * mat.C12 + C22 * mat.C22 + C23 * mat.C32;
  matNew.C23 = C20 * mat.C03 + C21 * mat.C13 + C22 * mat.C23 + C23 * mat.C33;
  matNew.C30 = C30 * mat.C00 + C31 * mat.C10 + C32 * mat.C20 + C33 * mat.C30;
  matNew.C31 = C30 * mat.C01 + C31 * mat.C11 + C32 * mat.C21 + C33 * mat.C31;
  matNew.C32 = C30 * mat.C02 + C31 * mat.C12 + C32 * mat.C22 + C33 * mat.C32;
  matNew.C33 = C30 * mat.C03 + C31 * mat.C13 + C32 * mat.C23 + C33 * mat.C33;
  return matNew;
}
void Matrix44::operator*=(const Real &sca)
{
  C00 *= sca;
  C01 *= sca;
  C02 *= sca;
  C03 *= sca;
  C10 *= sca;
  C11 *= sca;
  C12 *= sca;
  C13 *= sca;
  C20 *= sca;
  C21 *= sca;
  C22 *= sca;
  C23 *= sca;
  C30 *= sca;
  C31 *= sca;
  C32 *= sca;
  C33 *= sca;
}
void Matrix44::operator*=(const Matrix44 &mat)
{
  Matrix44 matNew;
  matNew.C00 = C00 * mat.C00 + C01 * mat.C10 + C02 * mat.C20 + C03 * mat.C30;
  matNew.C01 = C00 * mat.C01 + C01 * mat.C11 + C02 * mat.C21 + C03 * mat.C31;
  matNew.C02 = C00 * mat.C02 + C01 * mat.C12 + C02 * mat.C22 + C03 * mat.C32;
  matNew.C03 = C00 * mat.C03 + C01 * mat.C13 + C02 * mat.C23 + C03 * mat.C33;
  matNew.C10 = C10 * mat.C00 + C11 * mat.C10 + C12 * mat.C20 + C13 * mat.C30;
  matNew.C11 = C10 * mat.C01 + C11 * mat.C11 + C12 * mat.C21 + C13 * mat.C31;
  matNew.C12 = C10 * mat.C02 + C11 * mat.C12 + C12 * mat.C22 + C13 * mat.C32;
  matNew.C13 = C10 * mat.C03 + C11 * mat.C13 + C12 * mat.C23 + C13 * mat.C33;
  matNew.C20 = C20 * mat.C00 + C21 * mat.C10 + C22 * mat.C20 + C23 * mat.C30;
  matNew.C21 = C20 * mat.C01 + C21 * mat.C11 + C22 * mat.C21 + C23 * mat.C31;
  matNew.C22 = C20 * mat.C02 + C21 * mat.C12 + C22 * mat.C22 + C23 * mat.C32;
  matNew.C23 = C20 * mat.C03 + C21 * mat.C13 + C22 * mat.C23 + C23 * mat.C33;
  matNew.C30 = C30 * mat.C00 + C31 * mat.C10 + C32 * mat.C20 + C33 * mat.C30;
  matNew.C31 = C30 * mat.C01 + C31 * mat.C11 + C32 * mat.C21 + C33 * mat.C31;
  matNew.C32 = C30 * mat.C02 + C31 * mat.C12 + C32 * mat.C22 + C33 * mat.C32;
  matNew.C33 = C30 * mat.C03 + C31 * mat.C13 + C32 * mat.C23 + C33 * mat.C33;
  C00 = matNew.C00;
  C01 = matNew.C01;
  C02 = matNew.C02;
  C03 = matNew.C03;
  C10 = matNew.C10;
  C11 = matNew.C11;
  C12 = matNew.C12;
  C13 = matNew.C13;
  C20 = matNew.C20;
  C21 = matNew.C21;
  C22 = matNew.C22;
  C23 = matNew.C23;
  C30 = matNew.C30;
  C31 = matNew.C31;
  C32 = matNew.C32;
  C33 = matNew.C33;
}
std::ostream& operator<<(std::ostream &out, Matrix44 &mat)
{
  out << std::setprecision(7) << std::fixed;
  out << std::setw(10) << mat.C00 << ", " << std::setw(10) << mat.C01 << ", " << std::setw(10) << mat.C02 << ", " << std::setw(10) << mat.C03 << std::endl;
  out << std::setw(10) << mat.C10 << ", " << std::setw(10) << mat.C11 << ", " << std::setw(10) << mat.C12 << ", " << std::setw(10) << mat.C13 << std::endl;
  out << std::setw(10) << mat.C20 << ", " << std::setw(10) << mat.C21 << ", " << std::setw(10) << mat.C22 << ", " << std::setw(10) << mat.C23 << std::endl;
  out << std::setw(10) << mat.C30 << ", " << std::setw(10) << mat.C31 << ", " << std::setw(10) << mat.C32 << ", " << std::setw(10) << mat.C33 << std::endl;
  return out;
}

Matrix44 Matrix44::identity()
{
  Matrix44 mat;
  mat.C00 = mat.C11 = mat.C22 = mat.C33 = 1;
  mat.C01 = mat.C02 = mat.C03 = mat.C10 = mat.C12 = mat.C13 = mat.C20 = mat.C21 = mat.C23 = mat.C30 = mat.C31 = mat.C32 = 0;
  return mat;
}

//############# Matrix34 #############

Matrix34::Matrix34()
{

}

Matrix34::Matrix34(const Real value)
{
  set(value);
}

Matrix34::Matrix34(const Matrix44 &mat)
{
  C00 = mat.C00;
  C01 = mat.C01;
  C02 = mat.C02;
  C03 = mat.C03;
  C10 = mat.C10;
  C11 = mat.C11;
  C12 = mat.C12;
  C13 = mat.C13;
  C20 = mat.C20;
  C21 = mat.C21;
  C22 = mat.C22;
  C23 = mat.C23;
}

void Matrix34::operator=(const Matrix44 &mat)
{
  C00 = mat.C00;
  C01 = mat.C01;
  C02 = mat.C02;
  C03 = mat.C03;
  C10 = mat.C10;
  C11 = mat.C11;
  C12 = mat.C12;
  C13 = mat.C13;
  C20 = mat.C20;
  C21 = mat.C21;
  C22 = mat.C22;
  C23 = mat.C23;
}
Matrix34 Matrix34::operator+(const Matrix34 &mat) const
{
  Matrix34 matNew;
  matNew.C00 = C00 + mat.C00;
  matNew.C01 = C01 + mat.C01;
  matNew.C02 = C02 + mat.C02;
  matNew.C03 = C03 + mat.C03;
  matNew.C10 = C10 + mat.C10;
  matNew.C11 = C11 + mat.C11;
  matNew.C12 = C12 + mat.C12;
  matNew.C13 = C13 + mat.C13;
  matNew.C20 = C20 + mat.C20;
  matNew.C21 = C21 + mat.C21;
  matNew.C22 = C22 + mat.C22;
  matNew.C23 = C23 + mat.C23;
  return matNew;
}
Matrix34 Matrix34::operator-(const Matrix34 &mat) const
{
  Matrix34 matNew;
  matNew.C00 = C00 - mat.C00;
  matNew.C01 = C01 - mat.C01;
  matNew.C02 = C02 - mat.C02;
  matNew.C03 = C03 - mat.C03;
  matNew.C10 = C10 - mat.C10;
  matNew.C11 = C11 - mat.C11;
  matNew.C12 = C12 - mat.C12;
  matNew.C13 = C13 - mat.C13;
  matNew.C20 = C20 - mat.C20;
  matNew.C21 = C21 - mat.C21;
  matNew.C22 = C22 - mat.C22;
  matNew.C23 = C23 - mat.C23;
  return matNew;
}
void Matrix34::operator+=(const Matrix34 &mat)
{
  C00 += mat.C00;
  C01 += mat.C01;
  C02 += mat.C02;
  C03 += mat.C03;
  C10 += mat.C10;
  C11 += mat.C11;
  C12 += mat.C12;
  C13 += mat.C13;
  C20 += mat.C20;
  C21 += mat.C21;
  C22 += mat.C22;
  C23 += mat.C23;
}
void Matrix34::operator-=(const Matrix34 &mat)
{
  C00 -= mat.C00;
  C01 -= mat.C01;
  C02 -= mat.C02;
  C03 -= mat.C03;
  C10 -= mat.C10;
  C11 -= mat.C11;
  C12 -= mat.C12;
  C13 -= mat.C13;
  C20 -= mat.C20;
  C21 -= mat.C21;
  C22 -= mat.C22;
  C23 -= mat.C23;
}
Matrix34 Matrix34::operator*(const Real &sca) const
{
  Matrix34 matNew;
  matNew.C00 = C00 * sca;
  matNew.C01 = C01 * sca;
  matNew.C02 = C02 * sca;
  matNew.C03 = C03 * sca;
  matNew.C10 = C10 * sca;
  matNew.C11 = C11 * sca;
  matNew.C12 = C12 * sca;
  matNew.C13 = C13 * sca;
  matNew.C20 = C20 * sca;
  matNew.C21 = C21 * sca;
  matNew.C22 = C22 * sca;
  matNew.C23 = C23 * sca;
  return matNew;
}
PV3D Matrix34::operator*(const PV4D &pv) const
{
  PV3D pvNew;
  pvNew.C0 = C00 * pv.C0 + C01 * pv.C1 + C02 * pv.C2 + C03 * pv.C3;
  pvNew.C1 = C10 * pv.C0 + C11 * pv.C1 + C12 * pv.C2 + C13 * pv.C3;
  pvNew.C2 = C20 * pv.C0 + C21 * pv.C1 + C22 * pv.C2 + C23 * pv.C3;
  return pvNew;
}
void Matrix34::operator*=(const Real &sca)
{
  C00 *= sca;
  C01 *= sca;
  C02 *= sca;
  C03 *= sca;
  C10 *= sca;
  C11 *= sca;
  C12 *= sca;
  C13 *= sca;
  C20 *= sca;
  C21 *= sca;
  C22 *= sca;
  C23 *= sca;
}
std::ostream& operator<<(std::ostream &out, Matrix34 &mat)
{
  out << std::setprecision(7) << std::fixed;
  out << std::setw(10) << mat.C00 << ", " << std::setw(10) << mat.C01 << ", " << std::setw(10) << mat.C02 << ", " << std::setw(10) << mat.C03 << std::endl;
  out << std::setw(10) << mat.C10 << ", " << std::setw(10) << mat.C11 << ", " << std::setw(10) << mat.C12 << ", " << std::setw(10) << mat.C13 << std::endl;
  out << std::setw(10) << mat.C20 << ", " << std::setw(10) << mat.C21 << ", " << std::setw(10) << mat.C22 << ", " << std::setw(10) << mat.C23 << std::endl;
  return out;
}

void Matrix34::set(const Real value)
{
  C00 = value;
  C01 = value;
  C02 = value;
  C03 = value;
  C10 = value;
  C11 = value;
  C12 = value;
  C13 = value;
  C20 = value;
  C21 = value;
  C22 = value;
  C23 = value;
}
