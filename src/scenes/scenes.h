#ifndef SCENES_H_INCLUDED
#define SCENES_H_INCLUDED

#include <draw/drawstuff.h>
#include <build/heightfield.h>
#include <build/obstacles.h>

/* SCENES */

namespace Scenes
{
    void create_empty_world(void);

    void create_hurdles     (Obstacle&  obstacles);
    void create_shaky_ground(Obstacle&  obstacles);
    void create_hills       (Landscape& landscape);
    void create_stairways   (Obstacle&  obstacles);
    void create_plates      (Obstacle&  obstacles);
}

#endif // SCENES_H_INCLUDED
