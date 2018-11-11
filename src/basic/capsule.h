#ifndef CAPSULE_H_INCLUDED
#define CAPSULE_H_INCLUDED

struct Capsule {
    Capsule(unsigned d, double l, double r) : dir(d), len(l), rad(r) {}
    unsigned dir;
    double len, rad;
};

#endif // CAPSULE_H_INCLUDED
