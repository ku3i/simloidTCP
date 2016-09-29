#ifndef STANDARD_H_INCLUDED
#define STANDARD_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Standard {

const Vector3 size_lever(.025, .025, .5);

const double zheight_start  = 0.0;
const double joint_distance = 0.005;

void
create_pendulum(Robot& robot, bool with_accel_sensor = true)
{
    const Vector3 size_base (.5, .5, 0.75);

    dsPrint("PENDULUM\n");
    Vector3 pos;

    /* body */
    pos.x = .0;
    pos.y = 0.5*size_base.y;
    pos.z = 0.5*size_base.z + zheight_start;

    robot.create_box("base", pos, size_base, .0, constants::materials::heavy, colors::white, true, constants::friction::hi);

    /* arm */
    pos.y = -0.5*size_lever.y - joint_distance;
    pos.z = 0.9*size_base.z - 0.5 * size_lever.z + zheight_start;

    robot.create_box("lever", pos, size_lever, .0, constants::materials::heavy, colors::black, false, constants::friction::lo);

    /* connect joints */
    robot.connect_joint("base" , "lever", .0, .0, +0.5*size_lever.z - 0.5*size_lever.x, 'y', -180, +180, 90, JointType::normal, "joint0", "", 2);

    /* attach sensors */
    if (with_accel_sensor)
        robot.attach_accel_sensor("lever");

    /* camera */
    robot.set_camera_center_on("base");
    robot.setup_camera(Vector3(0.0, -1.2, .60), 90, 0, 0);
}

void
create_double_pendulum(Robot& robot)
{
    const Vector3 size_base (.5, .5, 1.25);

    dsPrint("DOUBLE PENDULUM\n");
    Vector3 pos;

    /* body */
    pos.x = .0;
    pos.y = 0.5*size_base.y;
    pos.z = 0.5*size_base.z + zheight_start;

    robot.create_box("base", pos, size_base, .0, constants::materials::heavy, colors::white, true, constants::friction::hi);

    /* arm */
    pos.y = -0.5*size_lever.y - joint_distance;
    pos.z = 0.9*size_base.z - 0.5 * size_lever.z + zheight_start;

    robot.create_box("lever1", pos, 1.001*size_lever, .0, constants::materials::body, colors::black, false, constants::friction::lo);

    pos.z = 0.9*size_base.z - 1.5 * size_lever.z + size_lever.x + zheight_start;

    robot.create_box("lever2", pos, size_lever, .0, constants::materials::body, colors::black, false, constants::friction::lo);


    /* connect joints */
    robot.connect_joint("base"   , "lever1", .0, .0, +0.5*size_lever.z - 0.5*size_lever.x, 'y', -180, +180, 0, JointType::normal, "joint0", "", 1);
    robot.connect_joint("lever1" , "lever2", .0, .0, +0.5*size_lever.z - 0.5*size_lever.x, 'y', -180, +180, 0, JointType::normal, "joint1", "", 1);

    /* attach sensors */
    robot.attach_accel_sensor("lever2");

    /* camera */
    robot.set_camera_center_on("base");
    robot.setup_camera(Vector3(0.0, -1.2, .60), 90, 0, 0);
}

void
create_rotor_horizontal(Robot& robot)
{
    const Vector3 size_base (.50, .50, .50);
    const Vector3 size_rotor(.50, .025, .025);

    dsPrint("HORIZONTAL EXCENTRIC ROTOR\n");
    Vector3 pos;

    /* body */
    pos.x = .0;
    pos.y = .0;
    pos.z = 0.5*size_base.z + zheight_start;

    robot.create_box("base", pos, size_base, .0, constants::materials::heavy, colors::white, true, constants::friction::hi);

    /* arm */
    pos.x = 0.5*size_rotor.x;
    pos.y = .0;
    pos.z = size_base.z + 0.5 * size_rotor.z + joint_distance + zheight_start;

    robot.create_box("rotor", pos, size_rotor, .0, constants::materials::body, colors::black, false, constants::friction::lo);

    /* connect joints */
    robot.connect_joint("base" , "rotor", -.5*size_rotor.x + .5*size_rotor.y, .0, .0, 'z', -180, +180, 0, JointType::normal, "joint0", "", 1);

    /* attach sensors */
    robot.attach_accel_sensor("rotor");

    /* camera */
    robot.set_camera_center_on("base");
    robot.setup_camera(Vector3(0.0, -1.20, 1.20), 90, -40, 0);
}

void
create_rotor_vertical(Robot& robot)
{
    const Vector3 size_base (.25, .25, .5);
    const Vector3 size_rotor(.50, .025, .025);

    dsPrint("VERTICAL CENTRIC ROTOR\n");
    Vector3 pos;

    /* body */
    pos.x = .0;
    pos.y = 0.5*size_base.y;
    pos.z = 0.5*size_base.z + zheight_start;

    robot.create_box("base", pos, size_base, .0, constants::materials::heavy, colors::white, true, constants::friction::hi);

    /* arm */
    pos.x = 0.0;
    pos.y = -0.5*size_rotor.y - joint_distance;
    pos.z = 0.9*size_base.z + zheight_start;

    robot.create_box("rotor", pos, size_rotor, .0, constants::materials::body, colors::black, false, constants::friction::lo);

    /* connect joints */
    robot.connect_joint("base" , "rotor", .0, .0, .0, 'y', -180, +180, 0, JointType::normal, "joint0", "", 1);

    /* attach sensors */
    robot.attach_accel_sensor("rotor");

    /* camera */
    robot.set_camera_center_on("base");
    robot.setup_camera(Vector3(0.0, -1.2, .60), 90, 0, 0);
}

void
create_rotor_axial(Robot& robot)
{
    const Vector3 size_base (.10, .50, .50);
    const Vector3 size_rotor(.10, .50, .10);

    dsPrint("AXIAL ROTOR\n");
    Vector3 pos;

    /* body */
    pos.x = .0;
    pos.y = .0;
    pos.z = 0.5*size_base.z + zheight_start;

    robot.create_box("base", pos, size_base, .0, constants::materials::heavy, colors::white, true, constants::friction::hi);

    /* arm */
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 1.0*size_base.z + 0.75*size_rotor.z + zheight_start;

    robot.create_box("rotor", pos, size_rotor, .0, constants::materials::body, colors::black, false, constants::friction::lo);

    /* connect joints */
    robot.connect_joint("base" , "rotor", .0, .0, .0, 'y', -180, +180, 0, JointType::normal, "joint0", "", 1);

    /* attach sensors */
    robot.attach_accel_sensor("rotor");

    /* camera */
    robot.set_camera_center_on("rotor");
    robot.setup_camera(Vector3(-0.45, -1.30, 0.90), 75, -15, 0);
}


}
}
#endif // STANDARD_H_INCLUDED
