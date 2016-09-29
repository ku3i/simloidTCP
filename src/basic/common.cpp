
#include "./common.h"

void common::normalizeVector3 (double v[3])
{
    double len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if (len <= 0.0) {
        v[0] = 1;
        v[1] = 0;
        v[2] = 0;
    }
    else {
        len = 1.0 / (double) sqrt(len);
        v[0] *= len;
        v[1] *= len;
        v[2] *= len;
    }
}

/* normal random variate generator
 * mean m, standard deviation s
 */
double common::box_muller(const double m, const double s, const double min, const double max)
{
    double x1, x2, w, y1;
    static double y2;
    static int use_last = 0;

    if (use_last) /* use value from previous call */
    {
        y1 = y2;
        use_last = 0;
    }
    else
    {
        do {
            x1 = 2.0 * getRandomDouble() - 1.0;
            x2 = 2.0 * getRandomDouble() - 1.0;
            w = x1 * x1 + x2 * x2;
        } while (w >= 1.0);

        w = sqrt( (-2.0 * log( w ) ) / w );
        y1 = x1 * w;
        y2 = x2 * w;
        use_last = 1;
    }

    double ret = ( m + y1 * s );
    if (ret < min) ret = min;
    if (ret > max) ret = max;
    return ret;
}
