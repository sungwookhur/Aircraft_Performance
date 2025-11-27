#pragma once
#include "Vec3.h"

namespace aero::math {

struct Mat3 {
    double m[3][3];

    Mat3() {
        for (int i=0;i<3;++i)
            for (int j=0;j<3;++j)
                m[i][j] = 0.0;
    }

    static Mat3 identity() {
        Mat3 I;
        I.m[0][0] = I.m[1][1] = I.m[2][2] = 1.0;
        return I;
    }
};

inline Vec3 operator*(const Mat3& A, const Vec3& v) {
    return Vec3(
        A.m[0][0]*v.x + A.m[0][1]*v.y + A.m[0][2]*v.z,
        A.m[1][0]*v.x + A.m[1][1]*v.y + A.m[1][2]*v.z,
        A.m[2][0]*v.x + A.m[2][1]*v.y + A.m[2][2]*v.z
    );
}

inline Mat3 operator*(const Mat3& A, const Mat3& B) {
    Mat3 C;
    for (int i=0;i<3;++i) {
        for (int j=0;j<3;++j) {
            C.m[i][j] = 0.0;
            for (int k=0;k<3;++k) {
                C.m[i][j] += A.m[i][k]*B.m[k][j];
            }
        }
    }
    return C;
}

inline Mat3 transpose(const Mat3& A) {
    Mat3 T;
    for (int i=0;i<3;++i)
        for (int j=0;j<3;++j)
            T.m[i][j] = A.m[j][i];
    return T;
}

} // namespace aero::math
