#ifndef STANDARD_H_INCLUDED
#define STANDARD_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Standard {

const Vector3 size_lever(.02, .015, .52);
const double weight_lever_kg = .133;

const double zheight_start  = 0.0;
const double joint_distance = 0.005;



const ActuatorParameters Sensorimotor {{/* bristle_displ_max = */ +8.71005098e-03, //+1.00000000e-03,
                                      /* bristle_stiffness = */ +9.93740632e-01, //+1.00348110e+00,
                                      /* sticking_friction = */ +2.17955433e-01, //+2.74093907e-01,
                                      /* coulomb_friction  = */ +1.04044079e-01, //+2.02936051e-01,
                                      /* fluid_friction    = */ +3.43298513e-01, //+1.81613953e-01,
                                      /* stiction_range    = */ +1.30738033e-01, //+3.39670766e-02,
                                      /* V_in              = */ +1.20000000e+01,
                                      /* kB                = */ +1.88657814e+00, //+1.95730358e+00,
                                      /* kM                = */ +1.88925820e+00, //+1.93830452e+00,
                                      /* R_i_inv           = */ +5.89352492e-02, //+8.19456429e-02
                                      }, /*assert_range=*/true };

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

    robot.create_box("lever", pos, size_lever, weight_lever_kg, /* Density Beech = 720*/ 0., colors::black, false, constants::friction::lo);

    /* connect joints */
    robot.connect_joint("base" , "lever", .0, .0, +0.5*size_lever.z - 0.03, 'y', -180, +180, 0, JointType::normal, "joint0", "", 5, Sensorimotor);

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


void
create_pendulum_gmes_ed(Robot& robot)
{
    const Vector3 base (.5, .5, 1.0);
    const Vector3 lever(.025, .025, .9);

    dsPrint("PENDULUM GMES Edition\n");
    Vector3 pos;

    /* body */
    pos.x = .0;
    pos.y = 0.5*base.y;
    pos.z = 0.5*base.z + zheight_start;

    robot.create_box("base", pos, base, .0, constants::materials::heavy, colors::white, true, constants::friction::hi);

    /* arm */
    pos.y = -0.5*lever.y - joint_distance;
    pos.z = 0.95*base.z - 0.5 * lever.z + zheight_start;

    robot.create_box("lever", pos, lever, .0, constants::materials::heavy, colors::black, false, constants::friction::lo);

    /* connect joints */
    robot.connect_joint("base" , "lever", .0, .0, +0.5*lever.z - 0.5*lever.x, 'y', -180, +180, 0, JointType::normal, "joint0", "", 5);

    /* camera */
    robot.set_camera_center_on("base");
    robot.setup_camera(Vector3(0.0, -1.2, .60), 90, 0, 0);
}

}} // namespace Robots::Standard

#endif // STANDARD_H_INCLUDED
