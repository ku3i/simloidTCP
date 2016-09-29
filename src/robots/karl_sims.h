#ifndef KARL_SIMS_H_INCLUDED
#define KARL_SIMS_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace KarlSims {

/* body */
const double length_body = 0.1;//.16;
const double radius_body = 0.05;//.11;

/* shoulders */
const double length_shoulder = .075;//.08;
const double radius_shoulder = .015;//.02;
const double joint_distance  = .01;

/* arms */
const double length_arms = .1;//.16;
const double radius_arms = .66*radius_body;//.04;

const double zheight_start = .05;

const unsigned int strength = 1;//3;

void
create_tadpole_0(Robot& robot)
{
    dsPrint("TADPOLE\n");

    double xpos, ypos, zpos;

    /* body */
    xpos = .0;
    ypos = radius_body + 0.5*length_body;
    zpos = radius_body + zheight_start;
    robot.create_segment("body", xpos, ypos, zpos, 2, length_body, radius_body,           0, constants::materials::normal, colors::white, true,  constants::friction::lo);

    zpos = 0.5*radius_body + zheight_start;
    ypos = radius_body + 0.20*length_body;
    /* shoulders */
    xpos = radius_body + radius_arms + joint_distance;
    robot.create_segment("lshd", +xpos, ypos, zpos, 2, length_shoulder, radius_shoulder,  0, constants::materials::body,   colors::black, false, constants::friction::lo);
    robot.create_segment("rshd", -xpos, ypos, zpos, 2, length_shoulder, radius_shoulder,  0, constants::materials::body,   colors::black, false, constants::friction::lo);

    /* arms */
    xpos = radius_body + radius_arms + joint_distance + 0.5*length_arms;
    robot.create_segment("larm", +xpos, ypos, zpos, 1, length_arms, radius_arms,          0, constants::materials::body,   colors::white, true,  constants::friction::hi);
    robot.create_segment("rarm", -xpos, ypos, zpos, 1, length_arms, radius_arms,          0, constants::materials::body,   colors::white, true,  constants::friction::hi);


    /* connect joints */
    robot.connect_joint("body", "lshd", .0, .0, .0,               'y', -90, +90, -10, JointType::normal   , "lshoulderroll" , ""              , strength);
    robot.connect_joint("body", "rshd", .0, .0, .0,               'Y', -90, +90, -10, JointType::symmetric, "rshoulderroll" , "lshoulderroll" , strength);

    robot.connect_joint("lshd", "larm", -0.5*length_arms, .0, .0, 'Z', -90, +90,   0, JointType::normal   , "lshoulderpitch", ""              , strength);
    robot.connect_joint("rshd", "rarm", +0.5*length_arms, .0, .0, 'z', -90, +90,   0, JointType::symmetric, "rshoulderpitch", "lshoulderpitch", strength);

    /* attach sensors */
    robot.attach_accel_sensor("body");

     /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(0.5, -0.45, .25), 120, 10, 0);
}

void
create_tadpole_1(Robot& robot)
{
    dsPrint("TADPOLE (alternative version)\n");

    double xpos, ypos, zpos;

    /* body */
    xpos = .0;
    ypos = radius_body + 0.5*length_body;
    zpos = radius_body + zheight_start;
    robot.create_segment("body",  xpos, ypos, zpos, 2, length_body    , radius_body    ,  0, constants::materials::normal, colors::white    , true , constants::friction::lo);

    zpos = 0.6*radius_body + zheight_start;
    ypos = radius_body + 0.25*length_body;
    /* shoulders */
    xpos = radius_body + radius_arms + joint_distance;
    robot.create_segment("lshd", +xpos, ypos, zpos, 2, length_shoulder, radius_shoulder,  0, constants::materials::body,   colors::invisible, false, constants::friction::lo);
    robot.create_segment("rshd", -xpos, ypos, zpos, 2, length_shoulder, radius_shoulder,  0, constants::materials::body,   colors::invisible, false, constants::friction::lo);

    /* arms */
    xpos = radius_body + radius_arms + joint_distance + 0.5*length_arms;
    robot.create_segment("larm", +xpos, ypos, zpos, 1, length_arms    , radius_arms    ,  0, constants::materials::body,   colors::white    , true , constants::friction::hi);
    robot.create_segment("rarm", -xpos, ypos, zpos, 1, length_arms    , radius_arms    ,  0, constants::materials::body,   colors::white    , true , constants::friction::hi);


    /* connect joints */
    robot.connect_joint("body", "lshd", .0, .0, .0,               'y', -90, +90, -20, JointType::normal   , "lshoulderroll" , ""              , strength);
    robot.connect_joint("body", "rshd", .0, .0, .0,               'Y', -90, +90, -20, JointType::symmetric, "rshoulderroll" , "lshoulderroll" , strength);

    robot.connect_joint("lshd", "larm", -0.5*length_arms, .0, .0, 'Z', -90, +90,   0, JointType::normal   , "lshoulderpitch", ""              , strength);
    robot.connect_joint("rshd", "rarm", +0.5*length_arms, .0, .0, 'z', -90, +90,   0, JointType::symmetric, "rshoulderpitch", "lshoulderpitch", strength);

    /* attach sensors */
    robot.attach_accel_sensor("body");

     /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(0.5, -0.45, .25), 120, 10, 0);
}

}} // namespace Robots::KarlSims


#endif // KARL_SIMS_H_INCLUDED
