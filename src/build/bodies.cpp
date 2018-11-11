#include "bodies.h"

dMass add(dMass const& m0, dMass const& m1) {
    dMass sum_mass;
    dMassSetZero(&sum_mass);
    dMassAdd(&sum_mass, &m0);
    dMassAdd(&sum_mass, &m1);
    return sum_mass;
}
