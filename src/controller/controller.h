#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <ode/ode.h>
#include <basic/common.h>
#include <build/robot.h>
#include <build/obstacles.h>
#include <build/heightfield.h>

class Controller
{
public:
    Controller(physics const& universe, Robot& robot, Obstacle& obstacles, Landscape& landscape, void (*_resetTime)())
    : universe(universe)
    , robot(robot)
    , obstacles(obstacles)
    , landscape(landscape)
    , reset_time(_resetTime)
    {}
    virtual ~Controller() {}
    virtual bool control(const double time) = 0;
    virtual void reset() = 0;

    bool is_paused(void) const { return paused; }

protected:
    physics const& universe;
    Robot&         robot;
    Obstacle&      obstacles;
    Landscape&     landscape;

    void (*reset_time)();
    bool paused;
};

#endif
