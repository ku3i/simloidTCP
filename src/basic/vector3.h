#ifndef VECTOR3_H_INCLUDED
#define VECTOR3_H_INCLUDED

#include <basic/common.h>

class Vector3
{
public:

    Vector3(const Vector3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {}
    Vector3(      Vector3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {}

    Vector3(void) : x(.0), y(.0), z(.0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vector3(double val) : x(val), y(val), z(val) {}
    Vector3(const double val[3]) : x(val[0]), y(val[1]), z(val[2]) {}

    Vector3& operator=(const Vector3& rhs) {
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
        return *this;
    }

    Vector3& operator=(Vector3& rhs) {
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
        return *this;
    }
    Vector3& operator=(const double& rhs) {
        this->x = rhs;
        this->y = rhs;
        this->z = rhs;
        return *this;
    }

    Vector3& operator+=(const Vector3& rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;
        return *this;
    }
    Vector3& operator-=(const Vector3& rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        this->z -= rhs.z;
        return *this;
    }
    Vector3& operator*=(const double& rhs)  {
        this->x *= rhs;
        this->y *= rhs;
        this->z *= rhs;
        return *this;
    }
    Vector3& operator/=(const double& rhs)  {
        this->x /= rhs;
        this->y /= rhs;
        this->z /= rhs;
        return *this;
    }
    void clip(double max_val) {
        x = common::clip(x, max_val);
        y = common::clip(y, max_val);
        z = common::clip(z, max_val);
    }
    void low_resolution() {
        x = common::low_resolution_sensor(x);
        y = common::low_resolution_sensor(y);
        z = common::low_resolution_sensor(z);
    }
    double length() const { return sqrt(x*x + y*y + z*z); }

    void normalize(void) {
        double l = 1.0 / length();
        x *= l;
        y *= l;
        z *= l;
    }
    double angle_phi  (void) const { return atan2(y,x); }
    double angle_theta(void) const { return atan2(z,x); }

    void random(double lower, double upper)
    {
        x = common::getRandomDouble(lower, upper);
        y = common::getRandomDouble(lower, upper);
        z = common::getRandomDouble(lower, upper);
    }

    void zero(void) { x = .0; y = .0; z = .0; }

    bool is_zero(void) const { return (x == .0 && y == .0 && z == .0); }

//    Vector3& operator=(Vector3 rhs) // the pass-by-value parameter serves as a temporary
//    {
//        rhs.swap(*this); // Non-throwing swap
//        return *this;
//    }
//
//    void swap(Vector3 &rhs) throw () // Also see non-throwing swap idiom
//    {
//       std::swap(this->x, rhs.x);
//       std::swap(this->y, rhs.y);
//       std::swap(this->z, rhs.z);
//    }

    double x, y, z;
};

inline Vector3 operator+(Vector3 lhs, const Vector3& rhs) {
    lhs += rhs;
    return lhs;
}
inline Vector3 operator-(Vector3 lhs, const Vector3& rhs) {
    lhs -= rhs;
    return lhs;
}
inline Vector3 operator*(Vector3 lhs, const double& rhs)  {
    lhs *= rhs;
    return lhs;
}
inline Vector3 operator*(const double& lhs, Vector3 rhs)  {
    rhs *= lhs;
    return rhs;
}
inline double operator*(const Vector3& lhs, const Vector3& rhs) { //scalar multiplication
    return lhs.x * rhs.x
         + lhs.y * rhs.y
         + lhs.z * rhs.z;
}
inline Vector3 operator/(Vector3 lhs, const double& rhs)  {
    lhs /= rhs;
    return lhs;
}
inline double distance(const Vector3& lhs, const Vector3& rhs) {
    return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x)
              + (lhs.y - rhs.y) * (lhs.y - rhs.y)
              + (lhs.z - rhs.z) * (lhs.z - rhs.z));
}
inline Vector3 clip(const Vector3& v, double max_val) {
    Vector3 result;
    result.x = common::clip(v.x, max_val);
    result.y = common::clip(v.y, max_val);
    result.z = common::clip(v.z, max_val);
    return result;
}

#endif // VECTOR3_H_INCLUDED
