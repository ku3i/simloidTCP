#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

#include <ode/ode.h>
#include <draw/drawstuff.h>
#include <basic/common.h>
#include <basic/constants.h>
#include <basic/vector3.h>

struct Camera_Setup
{
    Camera_Setup(Vector3 pos, double heading, double pitch, double roll)
    : pos(pos), heading(heading), pitch(pitch), roll(roll) {}

    Vector3 pos;
    double heading;
    double pitch;
    double roll;
};

class Camera
{
public:
    Camera()
    : rotate(false)
    , follow(true)
    , eta_t(constants::camera_parameters::fade_translation)
    , eta_r(constants::camera_parameters::fade_rotation)
    , rotation_velocity(constants::camera_parameters::rotation_velocity)
    {}

    void update(dBodyID const& camera_center_obj);
    void toggle_follow(dBodyID const& camera_center_obj);
    void toggle_rotate(void);

    void set_viewpoint(dBodyID const& camera_center_obj, Camera_Setup const& cam_setup);
    void get_viewpoint(float xyz[3], float hpr[3]);

    /* moving */
    void turn_cw(void);
    void turn_ccw(void);

    /* zooming */
    void zoom_in(void);
    void zoom_out(void);

    double lastPos[3];
    bool rotate;
    bool follow;

private:

    void set_fixed_position(void);
    void do_follow(dBodyID const& camera_center_obj);
    void do_rotate(dBodyID const& camera_center_obj);

    const double* center_position;
    double current_rotation_center[3];
    double rotation_distance;
    double follow_distance;
    double follow_angle;

    double fixed_position[3];

    const double eta_t;
    const double eta_r;

    double rotation_velocity;
    float vpos[3], vdir[3];

};

#endif // CAMERA_H_INCLUDED
