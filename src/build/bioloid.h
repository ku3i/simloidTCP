#ifndef _BIOLOID_H_
#define _BIOLOID_H_

#include <basic/common.h>
#include <basic/configuration.h>
#include <build/robot.h>
#include <build/physics.h>
#include <build/bodies.h>
#include <build/heightfield.h>
#include <build/obstacles.h>

extern Configuration global_conf;

namespace Bioloid
{
    /* generator routines */
    void create_robot(Robot& robot);
    void create_robot(Robot& robot, int index_number);
    void create_scene(Obstacle& obstacles, Landscape& landscape);
};

#endif

