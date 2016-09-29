#include <misc/camera.h>

void Camera::update(dBodyID camera_center_obj)
{
    dsGetViewpoint(vpos, vdir);

    if (rotate) do_rotate(camera_center_obj);
    else if (follow) do_follow(camera_center_obj);
    else set_fixed_position();

    dsSetViewpoint(vpos, vdir);
}

void Camera::do_follow(dBodyID camera_center_obj)
{
    const double* center = (const double *) dBodyGetPosition(camera_center_obj);

    const double vecx = -cos(follow_angle) * follow_distance;
    const double vecy = -sin(follow_angle) * follow_distance;

    vpos[0] = (1.0-eta_t) * vpos[0] + eta_t * (center[0] + vecx);
    vpos[1] = (1.0-eta_t) * vpos[1] + eta_t * (center[1] + vecy);

    const double dir = common::rad2deg(follow_angle);

    if (fabs(vdir[0]-dir) < 180.0) {
        vdir[0] = (1.0-eta_t) * vdir[0] + eta_t * dir;
    }
    else {
        vdir[0] = (1.0-eta_t) * vdir[0] + eta_t * dir + (1.0-eta_t) * 360.0;
    }

    /* follow pitch */
    const double dx = center_position[0] - vpos[0];
    const double dy = center_position[1] - vpos[1];
    const double dz = center_position[2] - vpos[2];

    const double follow_pitch = atan2(dz, sqrt(dx*dx + dy*dy));
    const double dir2 = common::rad2deg(follow_pitch);

    if (fabs(vdir[1]-dir2) < 180.0) {
        vdir[1] = (1.0-eta_t) * vdir[1] + eta_t * dir2;
    }
}

void Camera::set_fixed_position (void)
{
    vpos[0] = (1.0 - eta_t) * vpos[0] + eta_t * fixed_position[0];
    vpos[1] = (1.0 - eta_t) * vpos[1] + eta_t * fixed_position[1];
}

void Camera::do_rotate(dBodyID camera_center_obj)
{
    double phi;

    center_position = (const double *) dBodyGetPosition(camera_center_obj);

    for (unsigned int i = 0; i < 3; ++i)
        current_rotation_center[i] = (1.0 - eta_t) * current_rotation_center[i] + eta_t * center_position[i];

    double distance_left = common::dist3D(current_rotation_center, center_position);

    phi = atan2((vpos[1]-current_rotation_center[1]),(vpos[0]-current_rotation_center[0]));
    if (distance_left < 0.5) // start rotation when close enough
        phi += rotation_velocity;

    vpos[0] = (current_rotation_center[0] + cos(phi) * rotation_distance);
    vpos[1] = (current_rotation_center[1] + sin(phi) * rotation_distance);

    vdir[0] = 180.0 + 180.0 * phi / constants::m_pi;
    vdir[1] = 0.9 * vdir[1] + 0.1 * (-180.0 * atan2(vpos[2] - current_rotation_center[2] + 0.05, rotation_distance) / constants::m_pi);
}

void Camera::set_viewpoint(dBodyID camera_center_obj, Camera_Setup cam_setup)
{
    float xyz[3];
    xyz[0] = cam_setup.pos.x;
    xyz[1] = cam_setup.pos.y;
    xyz[2] = cam_setup.pos.z;

    float hpr[3];
    hpr[0] = cam_setup.heading;
    hpr[1] = cam_setup.pitch;
    hpr[2] = cam_setup.roll;

    dsSetViewpoint(xyz, hpr);
    center_position = (const double *) dBodyGetPosition(camera_center_obj);

    rotation_distance = sqrt((center_position[0]-xyz[0])*(center_position[0]-xyz[0]) + (center_position[1]-xyz[1])*(center_position[1]-xyz[1]));

    current_rotation_center[0] = xyz[0] + cos(M_PI*hpr[0]/180.0) * rotation_distance;
    current_rotation_center[1] = xyz[1] + sin(M_PI*hpr[0]/180.0) * rotation_distance;
    current_rotation_center[2] = xyz[2] + sin(M_PI*hpr[1]/180.0) * rotation_distance;

    const double dx = center_position[0] - xyz[0];
    const double dy = center_position[1] - xyz[1];

    follow_angle  = atan2(dy, dx);
    follow_distance = sqrt(dx*dx + dy*dy);

    fixed_position[0] = xyz[0];
    fixed_position[1] = xyz[1];
    fixed_position[2] = xyz[2];

    lastPos[0] = center_position[0];
    lastPos[1] = center_position[1];
    lastPos[2] = center_position[2];
}

void Camera::get_viewpoint(float xyz[3], float hpr[3])
{
    if (xyz) {
        xyz[0] = vpos[0];
        xyz[1] = vpos[1];
        xyz[2] = vpos[2];
    }
    if (hpr) {
        hpr[0] = vdir[0];
        hpr[1] = vdir[1];
        hpr[2] = vdir[2];
    }
}
void Camera::turn_cw(void)
{
    if (rotate) {
        rotation_velocity -= constants::camera_parameters::rate_turn_inc;
    } else if (follow) {
        follow_angle -= constants::camera_parameters::rate_turn;
        if (follow_angle < 0.0)
            follow_angle += 2.0 * constants::m_pi;
    }
    else {
        fixed_position[0] -= sin(common::deg2rad(vdir[0])) * 0.1;
        fixed_position[1] += cos(common::deg2rad(vdir[0])) * 0.1;
    }
}

void Camera::turn_ccw(void)
{
    if (rotate) {
        rotation_velocity += constants::camera_parameters::rate_turn_inc;
    } else if (follow) {
        follow_angle += constants::camera_parameters::rate_turn;
        if (follow_angle > 2 * constants::m_pi)
            follow_angle -= 2.0 * constants::m_pi;
    }
    else {
        fixed_position[0] += sin(common::deg2rad(vdir[0])) * 0.1;
        fixed_position[1] -= cos(common::deg2rad(vdir[0])) * 0.1;
    }
}

void Camera::zoom_in(void)
{
    if (rotate) {
        rotation_distance *= constants::camera_parameters::rate_zoom_in;
    }
    else if (follow) {
        follow_distance *= constants::camera_parameters::rate_zoom_in;
    }
    else {
        fixed_position[0] += cos(common::deg2rad(vdir[0])) * 0.1;
        fixed_position[1] += sin(common::deg2rad(vdir[0])) * 0.1;
    }
}

void Camera::zoom_out(void)
{
    if (rotate) {
        rotation_distance *= constants::camera_parameters::rate_zoom_out;
    }
    else if (follow) {
        follow_distance *= constants::camera_parameters::rate_zoom_out;
    }
    else {
        fixed_position[0] -= cos(common::deg2rad(vdir[0])) * 0.1;
        fixed_position[1] -= sin(common::deg2rad(vdir[0])) * 0.1;
    }
}

void Camera::toggle_rotate(void)
{
    rotate = !rotate;
    if (rotate) {
        current_rotation_center[0] = vpos[0] + cos(constants::m_pi * vdir[0] / 180.0) * rotation_distance;
        current_rotation_center[1] = vpos[1] + sin(constants::m_pi * vdir[0] / 180.0) * rotation_distance;
        current_rotation_center[2] = vpos[2] + sin(constants::m_pi * vdir[1] / 180.0) * rotation_distance;
    }
    else {
        fixed_position[0] = vpos[0];
        fixed_position[1] = vpos[1];
        fixed_position[2] = vpos[2];
    }
}

void Camera::toggle_follow(dBodyID camera_center_obj)
{
    follow = !follow;
    if (follow) {
        const double dx = vpos[0] - dBodyGetPosition(camera_center_obj)[0];
        const double dy = vpos[1] - dBodyGetPosition(camera_center_obj)[1];

        follow_angle = constants::m_pi + atan2(dy, dx);
        follow_distance = sqrt(dx*dx + dy*dy);
    }
    else {
        fixed_position[0] = vpos[0];
        fixed_position[1] = vpos[1];
        fixed_position[2] = vpos[2];
    }
}
