#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <ode/ode.h>
#include <basic/common.h>
#include <build/robot.h>
#include <build/obstacles.h>

class Controller
{
public:
    Controller(const physics& universe, Robot& robot, const Obstacle& obstacles, void (*_resetTime)())
    : universe(universe)
    , robot(robot)
    , obstacles(obstacles)
    , reset_time(_resetTime)
    {}
    virtual ~Controller() {}
    virtual bool control(const double time) = 0;
    virtual void reset() = 0;

    bool is_paused(void) const { return paused; }

protected:
    const physics&  universe;
          Robot&    robot;
    const Obstacle& obstacles;

    void (*reset_time)();
    bool paused;
};

#endif
