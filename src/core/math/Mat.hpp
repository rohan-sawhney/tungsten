#ifndef MAT_HPP_
#define MAT_HPP_

#include "Angle.hpp"
#include "Vec.hpp"
#include "Box.hpp"

#include "sampling/UniformSampler.hpp"

#include <array>

namespace Tungsten {

template<typename ElementType, uint32 Size>
class Mat
{
    typedef Vec<ElementType, Size> ColVec;

    union {
        ElementType _a[Size*Size];
        ColVec _v[Size];
    };

public:
    static constexpr uint32 ColsAtCompileTime = Size;
    static constexpr uint32 RowsAtCompileTime = Size;

    Mat() = default;

    template<typename... Ts>
    Mat(const ElementType &a, const Ts &... ts)
    : _a{a, ts...}
    {
    }

    inline ColVec operator*(const ColVec &v) const
    {
        ColVec result;
        for (unsigned i = 0; i < Size; ++i)
            result[i] = _v[i].dot(v);
        return result;
    }

    inline Mat operator*(const Mat &o) const
    {
        Mat result;
        for (unsigned i = 0; i < Size; ++i) {
            for (unsigned j = 0; j < Size; ++j) {
                ElementType dot = 0;
                for (unsigned k = 0; k < Size; ++k)
                    dot += operator()(i, k)*o(k, j);
                result(i, j) = dot;
            }
        }
        return result;
    }

    inline Mat operator*(ElementType a) const
    {
        Mat result;
        for (uint32 i = 0; i < Size*Size; ++i)
            result._a[i] = _a[i]*a;
        return result;
    }

    inline Box<ColVec, ElementType, Size> operator*(const Box<ColVec, ElementType, Size> &b) const
    {
        Box<ColVec, ElementType, Size> result;
        for (uint32 i = 0; i < (1 << Size); ++i) {
            ColVec v;
            for (uint32 j = 0; j < Size; ++j)
                v[j] = (i & (1 << j)) ? b.max()[j] : b.min()[j];

            result.grow(operator*(v));
        }
        return result;
    }

    inline Mat operator/(ElementType a) const
    {
        Mat result;
        for (uint32 i = 0; i < Size*Size; ++i)
            result._a[i] = _a[i]/a;
        return result;
    }

    inline Mat operator+(const Mat &o) const
    {
        Mat result;
        for (uint32 i = 0; i < Size*Size; ++i)
            result._a[i] = _a[i] + o._a[i];
        return result;
    }

    inline Mat operator-(const Mat &o) const
    {
        Mat result;
        for (uint32 i = 0; i < Size*Size; ++i)
            result._a[i] = _a[i] - o._a[i];
        return result;
    }

    inline Mat &operator+=(const Mat &o)
    {
        for (uint32 i = 0; i < Size*Size; ++i)
            _a[i] += o._a[i];
        return *this;
    }

    inline Mat &operator*=(const ElementType &a)
    {
        for (uint32 i = 0; i < Size*Size; ++i)
            _a[i] *= a;
        return *this;
    }

    inline Mat &operator/=(const ElementType &a)
    {
        for (uint32 i = 0; i < Size*Size; ++i)
            _a[i] /= a;
        return *this;
    }

    inline bool operator==(const Mat &o) const
    {
        for (uint32 i = 0; i < Size*Size; ++i)
            if (_a[i] == o[i])
                return true;
        return false;
    }

    inline bool operator!=(const Mat &o) const
    {
        for (uint32 i = 0; i < Size*Size; ++i)
            if (_a[i] != o[i])
                return true;
        return false;
    }

    inline Mat transpose() const
    {
        Mat result;
        for (unsigned i = 0; i < Size; ++i)
            for (unsigned j = 0; j < Size; ++j)
                result(i, j) = operator()(j, i);
        return std::move(result);
    }

    inline ElementType trace() const
    {
        ElementType result = operator()(0, 0);
        for (unsigned i = 1; i < Size; ++i)
            result += operator()(i, i);
        return std::move(result);
    }

    inline ElementType sum() const
    {
        ElementType result = _a[0];
        for (uint32 i = 1; i < Size*Size; ++i)
            result += _a[i];
        return result;
    }

    inline ElementType max() const
    {
        ElementType result = _a[0];
        for (uint32 i = 1; i < Size*Size; ++i)
            if (_a[i] > result)
                result = _a[i];
        return result;
    }

    inline ColVec col(int c) const
    {
        ColVec result;
        for (uint32 i = 0; i < Size; ++i)
            result[i] = operator()(i, c);
        return result;
    }

    inline ColVec row(int r) const
    {
        ColVec result;
        for (uint32 i = 0; i < Size; ++i)
            result[i] = operator()(r, i);
        return result;
    }

    inline void setCol(int c, const ColVec &v)
    {
        for (uint32 i = 0; i < Size; ++i)
            operator()(i, c) = v[i];
    }

    inline void setRow(int r, const ColVec &v)
    {
        for (uint32 i = 0; i < Size; ++i)
            operator()(r, i) = v[i];
    }

    inline ColVec diag() const
    {
        ColVec result;
        for (uint32 i = 0; i < Size; ++i)
            result[i] = operator()(i, i);
        return result;
    }

          ElementType &operator()(int i, int j)       { return _a[i*Size + j]; }
    const ElementType &operator()(int i, int j) const { return _a[i*Size + j]; }

          ElementType &operator[](int i)       { return _a[i]; }
    const ElementType &operator[](int i) const { return _a[i]; }

    static inline Mat zero()
    {
        Mat result;
        std::memset(&result._a[0], 0, Size*Size*sizeof(ElementType));
        return result;
    }

    static inline Mat identity()
    {
        Mat result = zero();
        for (unsigned i = 0; i < Size; ++i)
            result(i, i) = ElementType(1);
        return result;
    }

    static inline Mat diag(const Vec<ElementType, Size> &v)
    {
        Mat result = zero();
        for (unsigned i = 0; i < Size; ++i)
            result(i, i) = v[i];
        return result;
    }

    static inline Mat random(UniformSampler &rng)
    {
        Mat result;
        for (unsigned i = 0; i < Size*Size; ++i)
            result._a[i] = rng.next1D();
        return std::move(result);
    }

    friend std::ostream &operator<< (std::ostream &stream, const Mat &v) {
        for (uint32 i = 0; i < Size; ++i) {
            stream << '[';
            for (uint32 j = 0; j < Size; ++j)
                stream << v(i, j) << (j == Size - 1 ? "]" : ", ");
            if (i != Size - 1)
                stream << '\n';
        }
        return stream;
    }
};

template<typename ElementType, unsigned Size>
Mat<ElementType, Size> operator*(const ElementType &a, const Mat<ElementType, Size> &b)
{
    Mat<ElementType, Size> result;
    for (unsigned i = 0; i < Size; ++i)
        for (unsigned j = 0; j < Size; ++j)
            result(i, j) = a*b(i, j);
    return result;
}

template<typename ElementType, unsigned Size>
static inline Mat<ElementType, Size> outer(const Vec<ElementType, Size> &a, const Vec<ElementType, Size> &b)
{
    Mat<ElementType, Size> result;
    for (unsigned i = 0; i < Size; ++i)
        for (unsigned j = 0; j < Size; ++j)
            result(i, j) = a[i]*b[j];
    return result;
}

typedef Mat<float, 2> Mat2f;
typedef Mat<float, 3> Mat3f;
typedef Mat<double, 2> Mat2d;
typedef Mat<double, 3> Mat3d;
typedef Mat<double, 4> Mat4d;

static inline float offDiag(const Mat2f &m)
{
    return m(0, 1);
}

static inline Vec3f offDiag(const Mat3f &m)
{
    return Vec3f(m(1, 2), m(0, 2), m(0, 1));
}

static inline Mat2f rotMatrix(float w)
{
    float c = std::cos(w);
    float s = std::sin(w);
    return Mat2f(c, -s, s, c);
}

static inline Mat3f rotMatrix(Vec3f w)
{
    float theta = w.length();
    if (std::abs(theta) < 1e-8f)
        return Mat3f::identity();
    w /= theta;
    float s = std::sin(theta);
    float c = std::cos(theta);
    float c1 = 1.0f - c;
    float x = w.x(), y = w.y(), z = w.z();

    return Mat3f(
           c + x*x*c1,  x*y*c1 - z*s,  x*z*c1 + y*s,
         y*x*c1 + z*s,    c + y*y*c1,  y*z*c1 - x*s,
         z*x*c1 - y*s,  z*y*c1 + x*s,    c + z*z*c1
    );
}

static inline float curl(const Mat2f &m)
{
    return m(1, 0) - m(0, 1);
}

static inline Vec3f curl(const Mat3f &m)
{
    return Vec3f(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), m(1, 0) - m(0, 1));
}

namespace MatUtil {

template<typename T> Vec<T, 3> transformPoint(const Mat<T, 4> &m, const Vec<T, 3> &v)
{
    return Vec<T, 3>(
        m(0, 0)*v[0] + m(0, 1)*v[1] + m(0, 2)*v[2] + m(0, 3),
        m(1, 0)*v[0] + m(1, 1)*v[1] + m(1, 2)*v[2] + m(0, 3),
        m(2, 0)*v[0] + m(2, 1)*v[1] + m(2, 2)*v[2] + m(0, 3)
    );
}
template<typename T> Vec<T, 3> transformVector(const Mat<T, 4> &m, const Vec<T, 3> &v)
{
    return Vec<T, 3>(
        m(0, 0)*v[0] + m(0, 1)*v[1] + m(0, 2)*v[2],
        m(1, 0)*v[0] + m(1, 1)*v[1] + m(1, 2)*v[2],
        m(2, 0)*v[0] + m(2, 1)*v[1] + m(2, 2)*v[2]
    );
}

template<typename T> Mat<T, 4> extractRotation(const Mat<T, 4> &m)
{
    Mat<T, 4> result = Mat<T, 4>::zero();
    result.setCol(0, m.col(0).normalized());
    result.setCol(1, m.col(1).normalized());
    result.setCol(2, m.col(2).normalized());
    result(3, 3) = T(1);
    return std::move(result);
}
template<typename T> Mat<T, 4> extractTranslation(const Mat<T, 4> &m)
{
    Mat<T, 4> result = Mat<T, 4>::zero();
    result(0, 3) = m(0, 3);
    result(1, 3) = m(1, 3);
    result(2, 3) = m(2, 3);
    result(3, 3) = T(1);
    return std::move(result);
}
template<typename T> Mat<T, 4> extractScale(const Mat<T, 4> &m)
{
    Mat<T, 4> result = Mat<T, 4>::zero();
    result(0, 0) = m.col(0).length();
    result(1, 1) = m.col(1).length();
    result(2, 2) = m.col(2).length();
    result(3, 3) = T(1);
    return std::move(result);
}
template<typename T, uint32 Size> Vec<T, 3> extractRotationVec(Mat<T, Size> m)
{
    m = extractRotation(m);
    T theta, phi, psi;
    if (m(1, 2) <= -1) {
        theta = PI_HALF;
        phi = std::atan2(m(2, 0), m(2, 1));
        psi = 0;
    } else if (m(1, 2) >= 1.0f) {
        theta = -PI_HALF;
        phi = std::atan2(-m(2, 0), -m(2, 1));
        psi = 0;
    } else {
        theta = std::asin(m(1, 2));
        phi = std::atan2(m(1, 0), m(1, 1));
        psi = std::atan2(m(0, 2), m(2, 2));
    }
    return Vec<T, 3>(-theta, -psi, phi);
}
template<typename T> Vec<T, 3> extractTranslationVec(const Mat<T, 4> &m)
{
    return Vec<T, 3>(m(0, 3), m(1, 3), m(2, 3));
}
template<typename T, uint32 Size> Vec<T, 3> extractScaleVec(const Mat<T, Size> &m)
{
    return Vec<T, 3>(m.col(0).length(), m.col(1).length(), m.col(2).length());
}
template<typename T> Mat<T, 4> stripRotation(const Mat<T, 4> &m)
{
    Mat<T, 4> result = Mat<T, 4>::zero();
    result(0, 0) = m.col(0).length();
    result(1, 1) = m.col(1).length();
    result(2, 2) = m.col(2).length();
    result(0, 3) = m(0, 3);
    result(1, 3) = m(1, 3);
    result(2, 3) = m(2, 3);
    result(3, 3) = T(1);
    return std::move(result);
}
template<typename T> Mat<T, 4> stripScale(Mat<T, 4> m)
{
    m.setCol(0, m.col(0).normalized());
    m.setCol(1, m.col(1).normalized());
    m.setCol(2, m.col(2).normalized());
    return std::move(m);
}
template<typename T> Mat<T, 4> stripTranslation(Mat<T, 4> m)
{
    m(0, 3) = m(1, 3) = m(2, 3) = T(0);
    return std::move(m);
}

template<typename T> Mat<T, 4> translate(const Vec<T, 3> &v)
{
    Mat<T, 4> result = Mat<T, 4>::zero();
    result(0, 3) = v[0];
    result(1, 3) = v[1];
    result(2, 3) = v[2];
    result(3, 3) = T(1);
    return std::move(result);
}
template<typename T> Mat<T, 4> scale(const Vec<T, 3> &v)
{
    Mat<T, 4> result = Mat<T, 4>::zero();
    result(0, 0) = v[0];
    result(1, 1) = v[1];
    result(2, 2) = v[2];
    result(3, 3) = T(1);
    return std::move(result);
}

template<typename T> Mat<T, 4> rotXYZ(const Vec<T, 3> &r)
{
    T c[] = {std::cos(r.x()), std::cos(r.y()), std::cos(r.z())};
    T s[] = {std::sin(r.x()), std::sin(r.y()), std::sin(r.z())};

    return Mat<T, 4>(
        c[1]*c[2], -c[0]*s[2] + s[0]*s[1]*c[2],  s[0]*s[2] + c[0]*s[1]*c[2], T(0),
        c[1]*s[2],  c[0]*c[2] + s[0]*s[1]*s[2], -s[0]*c[2] + c[0]*s[1]*s[2], T(0),
            -s[1],                   s[0]*c[1],                   c[0]*c[1], T(0),
             T(0),                        T(0),                        T(0), T(1)
    );
}
template<typename T> Mat<T, 4> rotYXZ(const Vec<T, 3> &r)
{
    T c[] = {std::cos(r.x()), std::cos(r.y()), std::cos(r.z())};
    T s[] = {std::sin(r.x()), std::sin(r.y()), std::sin(r.z())};

    return Mat<T, 4>(
            c[1]*c[2] - s[1]*s[0]*s[2],   -c[1]*s[2] - s[1]*s[0]*c[2], -s[1]*c[0], T(0),
                             c[0]*s[2],                     c[0]*c[2],      -s[0], T(0),
            s[1]*c[2] + c[1]*s[0]*s[2],   -s[1]*s[2] + c[1]*s[0]*c[2],  c[1]*c[0], T(0),
                                  T(0),                          T(0),       T(0), T(1)
    );
}
template<typename T> Mat<T, 4> rotAxis(const Vec<T, 3> &axis, T angle)
{
    auto s = std::sin(angle);
    auto c = std::cos(angle);
    auto c1 = T(1) - c;
    auto x = axis.x();
    auto y = axis.y();
    auto z = axis.z();

    return Mat<T, 4>(
           c + x*x*c1,  x*y*c1 - z*s,  x*z*c1 + y*s, T(0),
         y*x*c1 + z*s,    c + y*y*c1,  y*z*c1 - x*s, T(0),
         z*x*c1 - y*s,  z*y*c1 + x*s,    c + z*z*c1, T(0),
                 T(0),          T(0),          T(0), T(1)
    );
}

template<typename T> Mat<T, 4> ortho(T l, T r, T b, T t, T n, T f)
{
    return Mat<T, 4>(
        2.0f/(r-l),       T(0),        T(0), -(r+l)/(r-l),
              T(0), 2.0f/(t-b),        T(0), -(t+b)/(t-b),
              T(0),       T(0), -2.0f/(f-n), -(f+n)/(f-n),
              T(0),       T(0),        T(0),          T(1)
    );
}
template<typename T> Mat<T, 4> perspective(T fov, T ratio, T nearZ, T farZ)
{
    auto t = T(1)/std::tan(fov*T(0.5f));
    auto a = (farZ + nearZ)/(farZ - nearZ);
    auto b = T(2)*farZ*nearZ/(farZ - nearZ);
    auto c = t/ratio;

    return Mat<T, 4>(
           c, T(0),  T(0), T(0),
        T(0),    t,  T(0), T(0),
        T(0), T(0),     a,   -b,
        T(0), T(0),  T(1), T(0)
    );
}
template<typename T> Mat<T, 4> lookAt(const Vec<T, 3> &pos, const Vec<T, 3> &target, const Vec<T, 3> &up)
{
    auto fwd = target - pos;
    auto f = fwd.normalized();
    auto r = f.cross(up).normalized();
    auto u = r.cross(f).normalized();

    return Mat<T, 4>(
        r.x(), u.x(), f.x(), pos.x(),
        r.y(), u.y(), f.y(), pos.y(),
        r.z(), u.z(), f.z(), pos.z(),
         T(0),  T(0),  T(0),    T(1)
    );
}

}

}

namespace std {

template<typename ElementType, unsigned Size>
Tungsten::Mat<ElementType, Size> abs(const Tungsten::Mat<ElementType, Size> &t)
{
    Tungsten::Mat<ElementType, Size> result;
    for (unsigned i = 0; i < Size; ++i)
        for (unsigned j = 0; j < Size; ++j)
            result(i, j) = std::abs(t(i, j));
    return result;
}

}

#endif /* MAT_HPP_ */
