#ifndef COLOR_H_INCLUDED
#define COLOR_H_INCLUDED

#include <draw/drawstuff.h>

struct Color4
{
    float r, g, b, a;

    Color4(void) : r(.0), g(.0), b(.0), a(1.0) {}
    Color4(float all) : r(all), g(all), b(all), a(1.0) {}
    Color4(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}
    Color4(float c[4]) : r(c[0]), g(c[1]), b(c[2]), a(c[3]) {}

    void set(float red, float green, float blue, float alpha) {
        r = red; g = green; b = blue; a = alpha;
    }

    bool operator==(Color4 const& cmp) const {
        return (r == cmp.r && g == cmp.g && b == cmp.b && a == cmp.a);
    }

    void apply() const { dsSetColorAlpha(r, g, b, a); }

    static Color4 set_transparency(const Color4& c, float alpha) {
        Color4 col = c;
        col.a = alpha;
        return col;
    }
};

namespace colors
{
    const Color4 black     { .3,  .3,  .4, 1.0};
    const Color4 gray      { .7,  .7,  .7, 1.0};
    const Color4 gray0     { .7,  .7,  .7, 0.5};
    const Color4 white     {1.0, 1.0, 1.0, 1.0};
    const Color4 white0    {1.0, 1.0, 1.0, 0.5};
    const Color4 yellow    {1.0, 1.0,  .0, 1.0};
    const Color4 redorange {1.0, 0.3,  .0, 1.0};
    const Color4 blue      { .0,  .0, 1.0, 1.0};
    const Color4 green     { .3, 1.0,  .0,  .9};
    const Color4 orange    {1.0,  .5,  .0, 1.0};
    const Color4 brown     { .6,  .4,  .3, 1.0};
    const Color4 pidgin    { .8,  .8, 1.0, 1.0};
    const Color4 cyan      { .0, 1.0, 1.0, 1.0};
    const Color4 magenta   {1.0,  .0, 1.0, 1.0};
    const Color4 invisible { .0,  .0,  .0,  .0};

    const Color4 light0    { .8,  .7,  .8, 1.0};
    const Color4 light1    { .7,  .8,  .8, 1.0};

    const Color4  orange_t {1.0,  .5,  .0, 0.7};
    const Color4    cyan_t { .0, 1.0, 1.0, 0.7};
    const Color4  yellow_t {1.0, 1.0,  .0, 0.7};
    const Color4 magenta_t {1.0,  .0, 1.0, 0.7};
    const Color4   black_t { .3,  .3,  .4, 0.5};

    const Color4  orange_l {1.0, .75, .25, 1.0};
    const Color4    cyan_l {.75, 1.0, 1.0, 1.0};
    const Color4  yellow_l {1.0, 1.0, .75, 1.0};
    const Color4 magenta_l {1.0, .75, 1.0, 1.0};

}

#endif // COLOR_H_INCLUDED
