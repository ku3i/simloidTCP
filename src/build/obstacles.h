#ifndef _OBSTACLES_H_
#define _OBSTACLES_H_

#include <string>
#include <ode/ode.h>

#include <basic/common.h>
#include <basic/vector3.h>
#include <build/physics.h>
#include <build/bodies.h>

/* This file contains the primitives
 * for obstacles and environmental objects */

class Obstacle {
public:
    Obstacle(const dWorldID &world, const dSpaceID &space)
    : world(world)
    , space(space)
    , objects(world, space, constants::max_obstacles)
    {
        dsPrint("Creating obstacles.\n");
    }
    const dWorldID&  world;
    const dSpaceID&  space;

    SolidVector objects;

    std::size_t number_of_objects() const { return objects.size(); }

    void create_box( std::string name
                   , const Vector3 pos
                   , const Vector3 len
                   , const dReal mass
                   , const dReal density
                   , const Color4 color
                   , dReal friction = dInfinity)
    {
        objects.create_box(name, pos, len, mass, density, color, true, friction);
    }

    void create_fixed_box(std::string name,
                          const Vector3 pos,
                          const Vector3 len,
                          const dReal mass,
                          const dReal density,
                          const Color4 color,
                          dReal friction = dInfinity)
    {
        objects.create_box(name, pos, len, mass, density, color, true, friction);
        objects[objects.size() - 1].fixed(world); // fix object last created
    }

    void print_statistics(void) const {
        dsPrint("   Obstacles: %d, Total Mass of Obstacles: %1.3f kg\n", number_of_objects(), objects.get_total_mass().mass);
    }

    ~Obstacle() {
        dsPrint("Destroying obstacles.\n");
    }

};

#endif

