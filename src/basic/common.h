#ifndef _COMMON_H_
#define _COMMON_H_

#include <cassert>
#include <cstdlib>
#include <math.h>

#include <basic/constants.h>

namespace common {

    void normalizeVector3 (double v[3]);

    // clipping of values
    template <typename T> inline T clip(T value, T min, T max);
    template <typename T> inline T clip(T value, T threshold);

    // radiant to -1..+1 (360°)
    inline double rad2norm(const double x) { return clip(x / constants::m_pi                          , 1.0); }
    inline double vel2norm(const double v) { return clip(v / constants::motor_parameter::max_joint_vel, 1.0); }

    inline double deg2rad (const double x) { return x * constants::m_pi / 180.0; }
    inline double rad2deg (const double x) { return x * 180.0 / constants::m_pi; }

    /* create uniformly distributed random values in range [min,max] */
    inline double getRandomDouble();
    inline double getRandomDouble(const double min, const double max);
    inline int    getRandomInt   (const int min, const int max);

    inline double rnd(double m, double s, double a)
    {
        assert(0.0 <= s and s <= 1.0);
        assert(0.0 <= a and a <= 1.0);
        const float rmin = (1 - a*s) * m;
        const float rmax = (1 + a*s) * m;

        assert(rmin >= 0. and rmax <= 2*m);
        return common::getRandomDouble(rmin, rmax); // consider to use non-uniform distribution, box muller here

    }

    inline double rnd(double a) { return common::getRandomDouble(-a, a); }


    /* create normal distributed random values
     * von http://www.taygeta.com/random/boxmuller.html
     * m - Mittelpunkt, s - Varianz
     */
    double box_muller(const double m, const double s, const double min, const double max);

    inline double grow(double m, double a) {
        assert(0.0 < a and a <= 1.0);
        return m*a;
    }

    inline double rndg(double m, double s, double r, double g) { return rnd( grow(m, g), s, r); }

    template <typename T> inline T dist3D(const T* v1, const T* v2);
    template <typename T> inline T dist2D(const T* v1, const T* v2);

    inline double low_resolution_sensor(double value);
    inline double avr_10bit_adc(double value);

    inline short  double2short(double value);
    inline double short2double(short  value);

    template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }
}

template <typename T>
inline T common::clip(T value, T min, T max)
{
    if (value > max) return value = max;
    else if (value < min) return value = min;
    else return value;
}

template <typename T>
inline T common::clip(T value, T threshold)
{
    if (value > threshold) return value = threshold;
    else if (value < -threshold) return value = -threshold;
    else return value;
}

inline double common::getRandomDouble(const double min, const double max)
{
    if (min > max) return getRandomDouble() * (min - max) + max;
    else return getRandomDouble() * (max - min) + min;
}

inline int common::getRandomInt(const int min, const int max)
{
    if (min > max) return rand() % (min - max + 1) + max;
    else return rand() % (max - min + 1) + min;
}

/* generates a pseudo-random double between 0.0 and 0.999...*/
inline double common::getRandomDouble()
{
    return (double) rand() / (double(RAND_MAX) + 1.0);
}

/* distance of two 3D points */
template <typename T>
inline T common::dist3D(const T* v1, const T* v2)
{
    return sqrt( (v1[0]-v2[0])*(v1[0]-v2[0]) +
                 (v1[1]-v2[1])*(v1[1]-v2[1]) +
                 (v1[2]-v2[2])*(v1[2]-v2[2]));
}

/* distance of two 2D points */
template <typename T>
inline T common::dist2D(const T* v1, const T* v2)
{
    return sqrt( (v1[0]-v2[0])*(v1[0]-v2[0]) +
                 (v1[1]-v2[1])*(v1[1]-v2[1]));
}

inline double common::low_resolution_sensor(double value)
{
    /* add Gaussian noise to sensor */
    double gaussian_noise = box_muller(0.0,
                                       constants::sensor_noise::std_dev,
                                       constants::sensor_noise::min_val,
                                       constants::sensor_noise::max_val);
    /* lower to 16 bit resolution */
    short s = double2short(value + gaussian_noise);
    return short2double(s); // and convert back to double
}

inline short common::double2short(double value) {
    value = clip(value, -1.0, 32767.0/32768.0);
    return static_cast<short>(value * 32768); // [-32768,32767]
}

inline double common::short2double(short value) {
    return static_cast<double>(value / 32768.0);
}

inline double common::avr_10bit_adc(double value)
{
    /* add Gaussian noise to ADC model */
    double gaussian_noise = box_muller(0.0,
                                       constants::avr_adc_10bit::std_dev,
                                       constants::avr_adc_10bit::min_val,
                                       constants::avr_adc_10bit::max_val);
    /* lower to 10 bit resolution */
    value = clip(value + gaussian_noise, -1.0, 511.0/512.0);
    value = static_cast<short>(512 * value);
    return value / 512.0;
}

#endif
