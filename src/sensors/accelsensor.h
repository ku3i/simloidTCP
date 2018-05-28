#ifndef _ACCELSENSOR_H_
#define _ACCELSENSOR_H_

#include <cmath>
#include <vector>
#include <ode/ode.h>

#include <draw/drawstuff.h>

#include <basic/common.h>
#include <basic/vector3.h>
#include <basic/configuration.h>


extern Configuration global_conf;

//TODO make to const Vector3
class axis_direction {
public:
    axis_direction(double x0, double x1, double x2)
    { x[0] = x0; x[1] = x1; x[2] = x2; }

    double x[3];
};

const axis_direction _left   (+1,  0,  0);
const axis_direction _right  (-1,  0,  0);
const axis_direction _up     ( 0,  0, +1);
const axis_direction _down   ( 0,  0, -1);
const axis_direction _back   ( 0, +1,  0);
const axis_direction _forward( 0, -1,  0);


class AccelSensor
{
public:
    AccelSensor(const dBodyID b, const axis_direction d0, const axis_direction d1, const axis_direction d2)
    : body_id(b)
    , dt(global_conf.step_length)
    , gravity(.0, .0, -constants::gravity)
    , acceleration(.0)
    , last_velocity(dBodyGetLinearVel(body_id))
    {
        for (unsigned int i = 0; i < 3; ++i) {
            dir[0][i] = d0.x[i]; // init directions
            dir[1][i] = d1.x[i];
            dir[2][i] = d2.x[i];
            common::normalizeVector3(dir[i]); // normalize
        }
    };

    const Vector3 update(void);

    void reset() {
        last_velocity = dBodyGetLinearVel(body_id);
        acceleration = 0.0;
    }
    void draw(void);

protected:
    dBodyID body_id;       // body to measure acceleration
    const double dt;       // timestep
    const Vector3 gravity;
    Vector3 acceleration;
    Vector3 last_velocity;
    double  dir[3][3];     // directions of sensor axes
};

class AccelVector
{
public:
    AccelVector(const std::size_t max_number_of_accels)
    : accels()
    , max_number_of_accels(max_number_of_accels)
    {
        dsPrint("Creating acceleration sensor vector...");
        accels.reserve(max_number_of_accels);
        dsPrint("done.\n");
    }

    ~AccelVector() { dsPrint("Destroying acceleration sensors.\n"); }

    unsigned int attach(const dBodyID body_id, const axis_direction d0, const axis_direction d1, const axis_direction d2)
    {
        unsigned int accel_id = accels.size();
        if (accel_id < max_number_of_accels) {
            accels.emplace_back(body_id, d0, d1, d2);
        } else {
            dsError("Exceeded maximum number of acceleration sensors %u.", max_number_of_accels);
        }

        return accel_id;
    }

          AccelSensor& operator[](std::size_t idx)       { return accels[idx]; }
    const AccelSensor& operator[](std::size_t idx) const { return accels[idx]; }

    std::size_t get_size() const { return accels.size(); }

    void reset_all(void) { for (auto& a : accels) a.reset(); }

    void destroy(void) {
        dsPrint("Destroying acceleration sensors (for recreation).\n");
        accels.clear();
    }

private:
    std::vector<AccelSensor> accels;
    const std::size_t        max_number_of_accels;

};

#endif
