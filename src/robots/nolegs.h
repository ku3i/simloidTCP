#ifndef NOLEGS_H_INCLUDED
#define NOLEGS_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Nolegs {

void create_worm(Robot& robot)
{
    dsPrint("WORM\n");

    const Vector3 len_head(.15,.15,.025);
    const Vector3 len_back(.15,.15,.025);
    const Vector3 len_tail(.15,.15,.025);

    const unsigned int max_segments = 5;
    const unsigned int torque = 5;
    const int max_angle = 60;

    const double zheight_start = 0.5*std::max(std::max(len_head.z, len_tail.z), len_back.z) + 0.001;
    Vector3 pos(.0, .0, zheight_start);

    /* head */
    pos.y = .5 * len_head.y;
    robot.create_box("head", pos, len_head, .0, constants::materials::body, colors::black, true, constants::friction::lo);

    /* back */
    pos.y = len_head.y + .5 * len_back.y;
    for (unsigned int idx = 0; idx < max_segments; ++idx)
    {
        pos.y = len_head.y + (0.5 + idx) * len_back.y;
        robot.create_box("back_" + std::to_string(idx), pos, len_back, .0, constants::materials::body, (idx%2==0)?colors::white:colors::pidgin, true, constants::friction::lo);
    }

    /* tail */
    pos.y = len_head.y + max_segments * len_back.y + .5*len_tail.y;
    robot.create_box("tail", pos, len_tail, .0, constants::materials::body, colors::black, true, constants::friction::lo);

    /* connect with joints */
    if (max_segments > 0){
        robot.connect_joint("head", "back_0", .0, -.5*len_back.y, .0, 'x', -max_angle, +max_angle, 0, JointType::normal, "joint_0", "", torque);

        for (unsigned int idx = 0; idx < max_segments-1; ++idx)
            robot.connect_joint("back_" + std::to_string(idx), "back_" + std::to_string(idx+1), .0, -.5*len_back.y, .0, 'x', -max_angle, +max_angle, 0, JointType::normal, "joint_", "", torque);

        robot.connect_joint("back_" + std::to_string(max_segments-1), "tail", .0, -.5*len_back.y, .0, 'x', -max_angle, +max_angle, 0, JointType::normal, "joint_" + std::to_string(max_segments), "", torque);
    }
    else /* connect head with tail */
        robot.connect_joint("head", "tail", .0, -.5*len_back.y, .0, 'x', -max_angle, +max_angle, 0, JointType::normal, "joint", "", torque);

    /* attach sensors */
    robot.attach_accel_sensor("head");

    /* camera */
    std::string cam_obj = (max_segments > 0)? "back_" + std::to_string(max_segments/2) : "head";
    robot.set_camera_center_on(cam_obj);
    robot.setup_camera(Vector3(0.5, -0.45, .25), 120, 10, 0);
}

}} // namespace Robots::Nolegs


#endif // NOLEGS_H_INCLUDED
