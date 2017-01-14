#ifndef HUMANOID_H_INCLUDED
#define HUMANOID_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Biped {

void
create_ostrich(Robot& robot)
{
    const double zheight_start = .350;

    /* head */
    const double len_head = .060;
    const double rad_head = .030;

    /* neck */
    const double len_neck = .160;
    const double rad_neck = .025;

    /* body */
    const double len_body = .150;
    const double rad_body = .05;

    /* legs upper */
    const double rad_leg_upper = .66*rad_body;
    const double len_leg_upper = .50*len_body*2 - rad_leg_upper;

    /* legs lower */
    const double len_leg_lower = 1.33*len_leg_upper;
    const double rad_leg_lower = 0.80*rad_leg_upper;

    /* shoulders */
    const double len_shoulder = .010;
    const double rad_shoulder = .025;

    const double joint_distance = .005;

    double xpos, ypos, zpos;


    /* head */
    xpos = .0;
    ypos = .0;
    zpos = zheight_start + len_neck;
    robot.create_segment("head", xpos, ypos, zpos, 2, len_head, rad_head, 0, constants::materials::body, colors::black, true, constants::friction::lo); // head

    /* neck */
    ypos = .5*len_head;
    zpos = zheight_start + 0.5*len_neck;
    robot.create_segment("neck", xpos, ypos, zpos, 3, len_neck, rad_neck, 0, constants::materials::body, colors::white, true, constants::friction::lo); // neck

    /* body */
    ypos = rad_body + .5*len_body;
    zpos = zheight_start;
    robot.create_segment("body", xpos, ypos, zpos, 2, len_body, rad_body, 0, constants::materials::normal, colors::black, true, constants::friction::lo); // body

    /* shoulders */
    zpos = zheight_start;
    ypos = rad_body + .37*len_body;
    xpos = rad_body + rad_leg_upper + joint_distance;
    robot.create_segment("lfsh", +xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::black, true, constants::friction::lo); // left fore shoulder
    robot.create_segment("rfsh", -xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::black, true, constants::friction::lo); // right fore shoulder

    /* fore legs upper */
    zpos = zheight_start - .5*len_leg_upper;
    robot.create_segment("lflu", +xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::lo); // left fore leg upper
    robot.create_segment("rflu", -xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::lo); // right fore leg upper

    /* fore legs lower */
    zpos = zheight_start - len_leg_upper - .5*len_leg_lower;
    robot.create_segment("lfll", +xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // left fore leg lower
    robot.create_segment("rfll", -xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // right fore leg lower

    /* connect by joints */
    robot.connect_joint("head", "neck", .0, .0, .5*len_neck,                'x', -45,  +45, -10, JointType::normal, "headpitch");
    robot.connect_joint("neck", "body", .0, -.5*len_body - .5*rad_body, .0, 'x', -45,  +90,  10, JointType::normal, "neckpitch");

    robot.connect_joint("body", "lfsh", .0, .0, .0,                'x', -90,  +90, -30, JointType::normal,    "lforehippitch");
    robot.connect_joint("body", "rfsh", .0, .0, .0,                'x', -90,  +90, -30, JointType::symmetric, "rforehippitch", "lforehippitch");

    robot.connect_joint("lfsh", "lflu", .0, .0, +.5*len_leg_upper, 'y', -90,  +90, +10, JointType::normal,    "lforehiproll");
    robot.connect_joint("rfsh", "rflu", .0, .0, +.5*len_leg_upper, 'Y', -90,  +90, +10, JointType::symmetric, "rforehiproll", "lforehiproll");
    robot.connect_joint("lflu", "lfll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::normal,    "lforeelbpitch");
    robot.connect_joint("rflu", "rfll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::symmetric, "rforeelbpitch", "lforeelbpitch");

    /* attach sensors */
    robot.attach_accel_sensor("head");

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(0.5, -0.45, .25), 120, 10, 0);
}

void
create_humanoid0(Robot& robot)
{
    /* head */
    const double length_head = 0.02;
    const double radius_head = 0.05;

    /* body */
    const double length_upper_torso = .100;
    const double radius_upper_torso = .05;

    const double length_lower_torso = .01;
    const double radius_lower_torso = .05;

    /* shoulders */
    const double length_shoulder = .040;
    const double radius_shoulder = .010;
    const double arm_torso_distance = 0.005;

    /* arms upper */
    const double length_arm_upper = .095;
    const double radius_arm_upper = .025;

    /* arms lower */
    const double length_arm_lower = .100;
    const double radius_arm_lower = .020;

    /* hip */
    const double length_hip = .050;
    const double radius_hip = .010;

    /* legs upper */
    const double length_leg_upper = .120;
    const double radius_leg_upper = .030;

    /* legs lower */
    const double length_leg_lower = .120;
    const double radius_leg_lower = .025;
    const double leg_distance = .01;

    /* ankles */
    const double length_ankles = .010;
    const double radius_ankles = .025;

    /* feet */
    const double length_feet = 0.1;
    const double width_feet = 0.05;
    const double height_feet = 0.025;

    const double zheight_start = .55;

    Vector3 pos(.0);

    /* body */
    pos.z = zheight_start - radius_upper_torso - 0.5*length_upper_torso;
    robot.create_segment("upto", pos.x, pos.y, pos.z, 3, length_upper_torso, radius_upper_torso, 0, constants::materials::normal, colors::white, true);

    pos.z = zheight_start - radius_upper_torso - length_upper_torso - .5*length_lower_torso;
    robot.create_segment("loto", pos.x, pos.y, pos.z, 3, length_lower_torso, radius_lower_torso, 0, constants::materials::normal, colors::white, false);

    pos.z = zheight_start + radius_upper_torso + 0.5*length_head;
    robot.create_segment("head", pos.x, pos.y, pos.z, 3, length_head, radius_head, 0, constants::materials::normal, colors::black, true);

    /* shoulders */
    pos.z = zheight_start - radius_upper_torso;
    pos.y = .0;
    pos.x = radius_upper_torso + radius_arm_upper + arm_torso_distance;

    robot.create_segment("lshd", +pos.x, pos.y, pos.z, 2, length_shoulder, radius_shoulder, 0, constants::materials::heavy, colors::black, 0);
    robot.create_segment("rshd", -pos.x, pos.y, pos.z, 2, length_shoulder, radius_shoulder, 0, constants::materials::heavy, colors::black, 0);

    /* hips */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_segment("lhip", +pos.x, pos.y, pos.z, 2, length_hip, radius_hip, 0, constants::materials::heavy, colors::black, 0);
    robot.create_segment("rhip", -pos.x, pos.y, pos.z, 2, length_hip, radius_hip, 0, constants::materials::heavy, colors::black, 0);

    /* arms upper */
    pos.z = zheight_start - radius_upper_torso - 0.25*length_arm_upper;
    pos.y = .0;
    pos.x = radius_upper_torso + radius_arm_upper + arm_torso_distance;
    robot.create_segment("lau0", +pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rau0", -pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);

    pos.z = zheight_start - radius_upper_torso - 0.75*length_arm_upper;
    robot.create_segment("lau1", +pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rau1", -pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);

    /* arms lower */
    pos.z = zheight_start - radius_upper_torso - length_arm_upper - .5*length_arm_lower;
    pos.y = .0;
    pos.x = radius_upper_torso + radius_arm_upper + arm_torso_distance;

    robot.create_segment("lalo", +pos.x, pos.y, pos.z, 3, length_arm_lower, radius_arm_lower, 0, constants::materials::heavy, colors::black, true);
    robot.create_segment("ralo", -pos.x, pos.y, pos.z, 3, length_arm_lower, radius_arm_lower, 0, constants::materials::heavy, colors::black, true);

    /* legs upper */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper -.25*length_leg_upper;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;
    robot.create_segment("llu0", +pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rlu0", -pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::white, true);

    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper -.75*length_leg_upper;
    robot.create_segment("llu1", +pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::blue , true);
    robot.create_segment("rlu1", -pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::blue , true);


    /* legs lower */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper - length_leg_upper - 0.5*length_leg_lower;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_segment("lllo", +pos.x, pos.y, pos.z, 3, length_leg_lower, radius_leg_lower, 0, constants::materials::heavy, colors::black, true);
    robot.create_segment("rllo", -pos.x, pos.y, pos.z, 3, length_leg_lower, radius_leg_lower, 0, constants::materials::heavy, colors::black, true);

    /* ankles */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper - length_leg_upper - length_leg_lower - radius_leg_lower - length_ankles;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_segment("lean", +pos.x, pos.y, pos.z, 2, length_ankles, radius_ankles, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rian", -pos.x, pos.y, pos.z, 2, length_ankles, radius_ankles, 0, constants::materials::heavy, colors::white, true);

    /* feet */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper - length_leg_upper - length_leg_lower - radius_ankles - height_feet;
    pos.y = -0.5*length_feet + radius_leg_lower;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_box("left", +pos.x, pos.y, pos.z, width_feet, length_feet, height_feet, 0, constants::materials::heavy, colors::gray, true);
    robot.create_box("rift", -pos.x, pos.y, pos.z, width_feet, length_feet, height_feet, 0, constants::materials::heavy, colors::gray, true);


    /* connect with joints */
    robot.connect_joint("upto", "head", .0, .0, -.5*length_head-0.5*radius_head, 'x', -60, +45, -10, JointType::normal, "neck",       "");
    robot.connect_joint("upto", "loto", .0, .0, +.5*length_lower_torso,          'x', -30, +90,   0, JointType::normal, "waistpitch", "");

    robot.connect_joint("upto", "lshd", .0, .0, .0,                    'x', - 90, + 90, +10, JointType::normal   , "lshoulderpitch", ""              ); // 180 deg is not enough
    robot.connect_joint("upto", "rshd", .0, .0, .0,                    'x', - 90, + 90, +10, JointType::symmetric, "rshoulderpitch", "lshoulderpitch");

    robot.connect_joint("lshd", "lau0", .0, .0, +.25*length_arm_upper, 'y', - 90, +150, +10, JointType::normal   , "lshoulderroll" , ""              );
    robot.connect_joint("rshd", "rau0", .0, .0, +.25*length_arm_upper, 'Y', - 90, +150, +10, JointType::symmetric, "rshoulderroll" , "lshoulderroll" );

    robot.connect_joint("lau0", "lau1", .0, .0, +.25*length_arm_upper, 'Z', - 90, + 90,   0, JointType::normal   , "lshoulderyaw"  , ""              );
    robot.connect_joint("rau0", "rau1", .0, .0, +.25*length_arm_upper, 'z', - 90, + 90,   0, JointType::symmetric, "rshoulderyaw"  , "lshoulderyaw"  );

    robot.connect_joint("lau1", "lalo", .0, .0, +.5*length_arm_lower , 'x',    0, +120, +40, JointType::normal   , "lelbowpitch"   , ""              );
    robot.connect_joint("rau1", "ralo", .0, .0, +.5*length_arm_lower , 'x',    0, +120, +40, JointType::symmetric, "relbowpitch"   , "lelbowpitch"   );


    robot.connect_joint("loto", "lhip", .0, .0, .0                   , 'x', - 90, + 90, +20, JointType::normal   , "lhippitch" , ""          );
    robot.connect_joint("loto", "rhip", .0, .0, .0                   , 'x', - 90, + 90, +20, JointType::symmetric, "rhippitch" , "lhippitch" );

    robot.connect_joint("lhip", "llu0", .0, .0, +.25*length_leg_upper, 'y', - 90, + 90,   0, JointType::normal,    "lhiproll"  , ""          );
    robot.connect_joint("rhip", "rlu0", .0, .0, +.25*length_leg_upper, 'Y', - 90, + 90,   0, JointType::symmetric, "rhiproll"  , "lhiproll"  );

    robot.connect_joint("llu0", "llu1", .0, .0, +.25*length_leg_upper, 'Z', - 30, + 60,   0, JointType::normal,    "lhipyaw"   , ""          );
    robot.connect_joint("rlu0", "rlu1", .0, .0, +.25*length_leg_upper, 'z', - 30, + 60,   0, JointType::symmetric, "rhipyaw"   , "lhipyaw"   );

    robot.connect_joint("llu1", "lllo", .0, .0, +.50*length_leg_lower, 'x', -120,    0, -20, JointType::normal,    "lkneepitch", ""          );
    robot.connect_joint("rlu1", "rllo", .0, .0, +.50*length_leg_lower, 'x', -120,    0, -20, JointType::symmetric, "rkneepitch", "lkneepitch");

    robot.connect_joint("lllo", "lean", .0, .0, +.5*radius_ankles                          , 'x', -45, +45, +5, JointType::normal,    "lanklepitch", "");
    robot.connect_joint("rllo", "rian", .0, .0, +.5*radius_ankles                          , 'x', -45, +45, +5, JointType::symmetric, "ranklepitch", "lanklepitch");
    robot.connect_joint("lean", "left", .0, 0.5*length_feet - radius_leg_lower, height_feet, 'y', -45, +45,   0, JointType::normal,    "lankleroll", "");
    robot.connect_joint("rian", "rift", .0, 0.5*length_feet - radius_leg_lower, height_feet, 'Y', -45, +45,   0, JointType::symmetric, "rankleroll", "lankleroll");

    /* attach sensors */
    robot.attach_accel_sensor("head");

    /* camera */
    robot.set_camera_center_on("upto");
    robot.setup_camera(Vector3(0.76, -0.78, .55), 135, -5, 0);

}

void
create_humanoid1(Robot& robot)
{
    /* head */
    const double length_head = 0.02;
    const double radius_head = 0.05;

    /* body */
    const double length_upper_torso = .100;
    const double radius_upper_torso = .05;

    const double length_middle_torso = .01;
    const double radius_middle_torso = .04;

    const double length_lower_torso = .01;
    const double radius_lower_torso = .05;

    /* shoulders */
    const double length_shoulder = .040;
    const double radius_shoulder = .010;
    const double arm_torso_distance = 0.005;

    /* arms upper */
    const double length_arm_upper = .095;
    const double radius_arm_upper = .025;

    /* arms lower */
    const double length_arm_lower = .100;
    const double radius_arm_lower = .020;

    /* hip */
    const double length_hip = .050;
    const double radius_hip = .010;

    /* legs upper */
    const double length_leg_upper = .120;
    const double radius_leg_upper = .030;

    /* legs lower */
    const double length_leg_lower = .120;
    const double radius_leg_lower = .025;
    const double leg_distance = .01;

    /* ankles */
    const double length_ankles = .010;
    const double radius_ankles = .025;

    /* feet */
    const double length_feet = 0.09;
    const double width_feet = 0.05;
    const double height_feet = 0.025;

    const double zheight_start = .55;

    Vector3 pos(.0);

    /* body */
    pos.z = zheight_start - radius_upper_torso - 0.5*length_upper_torso;
    robot.create_segment("upto", pos.x, pos.y, pos.z, 3, length_upper_torso, radius_upper_torso, 0, constants::materials::normal, colors::white, true);

    pos.z = zheight_start - radius_upper_torso - length_upper_torso - .5*length_middle_torso;
    robot.create_segment("mito", pos.x, pos.y, pos.z, 3, length_middle_torso, radius_middle_torso, 0, constants::materials::normal, colors::black, false);

    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_middle_torso - .5*length_lower_torso;
    robot.create_segment("loto", pos.x, pos.y, pos.z, 3, length_lower_torso, radius_lower_torso, 0, constants::materials::normal, colors::white, false);

    pos.z = zheight_start + radius_upper_torso + 0.5*length_head;
    robot.create_segment("head", pos.x, pos.y, pos.z, 3, length_head, radius_head, 0, constants::materials::normal, colors::black, true);

    /* shoulders */
    pos.z = zheight_start - radius_upper_torso;
    pos.y = .0;
    pos.x = radius_upper_torso + radius_arm_upper + arm_torso_distance;

    robot.create_segment("lshd", +pos.x, pos.y, pos.z, 2, length_shoulder, radius_shoulder, 0, constants::materials::heavy, colors::black, 0);
    robot.create_segment("rshd", -pos.x, pos.y, pos.z, 2, length_shoulder, radius_shoulder, 0, constants::materials::heavy, colors::black, 0);

    /* hips */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_segment("lhip", +pos.x, pos.y, pos.z, 2, length_hip, radius_hip, 0, constants::materials::heavy, colors::black, 0);
    robot.create_segment("rhip", -pos.x, pos.y, pos.z, 2, length_hip, radius_hip, 0, constants::materials::heavy, colors::black, 0);

    /* arms upper */
    pos.z = zheight_start - radius_upper_torso - 0.25*length_arm_upper;
    pos.y = .0;
    pos.x = radius_upper_torso + radius_arm_upper + arm_torso_distance;
    robot.create_segment("lau0", +pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rau0", -pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);

    pos.z = zheight_start - radius_upper_torso - 0.75*length_arm_upper;
    robot.create_segment("lau1", +pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rau1", -pos.x, pos.y, pos.z, 3, 0.5*length_arm_upper, radius_arm_upper, 0, constants::materials::heavy, colors::white, true);

    /* arms lower */
    pos.z = zheight_start - radius_upper_torso - length_arm_upper - .5*length_arm_lower;
    pos.y = .0;
    pos.x = radius_upper_torso + radius_arm_upper + arm_torso_distance;

    robot.create_segment("lalo", +pos.x, pos.y, pos.z, 3, length_arm_lower, radius_arm_lower, 0, constants::materials::heavy, colors::black, true);
    robot.create_segment("ralo", -pos.x, pos.y, pos.z, 3, length_arm_lower, radius_arm_lower, 0, constants::materials::heavy, colors::black, true);

    /* legs upper */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper -.25*length_leg_upper;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;
    robot.create_segment("llu0", +pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rlu0", -pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::white, true);

    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper -.75*length_leg_upper;
    robot.create_segment("llu1", +pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::blue , true);
    robot.create_segment("rlu1", -pos.x, pos.y, pos.z, 3, 0.5*length_leg_upper, radius_leg_upper, 0, constants::materials::heavy, colors::blue , true);


    /* legs lower */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper - length_leg_upper - 0.5*length_leg_lower;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_segment("lllo", +pos.x, pos.y, pos.z, 3, length_leg_lower, radius_leg_lower, 0, constants::materials::heavy, colors::black, true);
    robot.create_segment("rllo", -pos.x, pos.y, pos.z, 3, length_leg_lower, radius_leg_lower, 0, constants::materials::heavy, colors::black, true);

    /* ankles */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper - length_leg_upper - length_leg_lower - radius_leg_lower - length_ankles;
    pos.y = .0;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_segment("lean", +pos.x, pos.y, pos.z, 2, length_ankles, radius_ankles, 0, constants::materials::heavy, colors::white, true);
    robot.create_segment("rian", -pos.x, pos.y, pos.z, 2, length_ankles, radius_ankles, 0, constants::materials::heavy, colors::white, true);

    /* feet */
    pos.z = zheight_start - radius_upper_torso - length_upper_torso - length_lower_torso - radius_lower_torso - radius_leg_upper - length_leg_upper - length_leg_lower - radius_ankles - height_feet;
    pos.y = -0.5*length_feet + radius_leg_lower;
    pos.x = leg_distance + radius_leg_lower;

    robot.create_box("left", +pos.x, pos.y, pos.z, width_feet, length_feet, height_feet, 0, constants::materials::heavy, colors::gray, true);
    robot.create_box("rift", -pos.x, pos.y, pos.z, width_feet, length_feet, height_feet, 0, constants::materials::heavy, colors::gray, true);


    /* connect with joints */
    robot.connect_joint("upto", "head", .0, .0, -.5*length_head-0.5*radius_head, 'x', -60, +45, -10, JointType::normal, "neck",       "");
    robot.connect_joint("upto", "mito", .0, .0, +.5*length_middle_torso,         'y', -45, +45,   0, JointType::normal, "waistroll" , "");
    robot.connect_joint("mito", "loto", .0, .0, +.5*length_lower_torso,          'x', -30, +90,   0, JointType::normal, "waistpitch", "");

    robot.connect_joint("upto", "lshd", .0, .0, .0,                    'x', - 90, + 90, +10, JointType::normal   , "lshoulderpitch", ""              ); // 180 deg is not enough
    robot.connect_joint("upto", "rshd", .0, .0, .0,                    'x', - 90, + 90, +10, JointType::symmetric, "rshoulderpitch", "lshoulderpitch");

    robot.connect_joint("lshd", "lau0", .0, .0, +.25*length_arm_upper, 'y', - 90, +150, +10, JointType::normal   , "lshoulderroll" , ""              );
    robot.connect_joint("rshd", "rau0", .0, .0, +.25*length_arm_upper, 'Y', - 90, +150, +10, JointType::symmetric, "rshoulderroll" , "lshoulderroll" );

    robot.connect_joint("lau0", "lau1", .0, .0, +.25*length_arm_upper, 'Z', - 90, + 90,   0, JointType::normal   , "lshoulderyaw"  , ""              );
    robot.connect_joint("rau0", "rau1", .0, .0, +.25*length_arm_upper, 'z', - 90, + 90,   0, JointType::symmetric, "rshoulderyaw"  , "lshoulderyaw"  );

    robot.connect_joint("lau1", "lalo", .0, .0, +.5*length_arm_lower , 'x',    0, +120, +40, JointType::normal   , "lelbowpitch"   , ""              );
    robot.connect_joint("rau1", "ralo", .0, .0, +.5*length_arm_lower , 'x',    0, +120, +40, JointType::symmetric, "relbowpitch"   , "lelbowpitch"   );


    robot.connect_joint("loto", "lhip", .0, .0, .0                   , 'x', - 90, + 90, +20, JointType::normal   , "lhippitch" , ""          );
    robot.connect_joint("loto", "rhip", .0, .0, .0                   , 'x', - 90, + 90, +20, JointType::symmetric, "rhippitch" , "lhippitch" );

    robot.connect_joint("lhip", "llu0", .0, .0, +.25*length_leg_upper, 'y', - 90, + 90,   0, JointType::normal,    "lhiproll"  , ""          );
    robot.connect_joint("rhip", "rlu0", .0, .0, +.25*length_leg_upper, 'Y', - 90, + 90,   0, JointType::symmetric, "rhiproll"  , "lhiproll"  );

    robot.connect_joint("llu0", "llu1", .0, .0, +.25*length_leg_upper, 'Z', - 30, + 60,   0, JointType::normal,    "lhipyaw"   , ""          );
    robot.connect_joint("rlu0", "rlu1", .0, .0, +.25*length_leg_upper, 'z', - 30, + 60,   0, JointType::symmetric, "rhipyaw"   , "lhipyaw"   );

    robot.connect_joint("llu1", "lllo", .0, .0, +.50*length_leg_lower, 'x', -120,    0, -20, JointType::normal,    "lkneepitch", ""          );
    robot.connect_joint("rlu1", "rllo", .0, .0, +.50*length_leg_lower, 'x', -120,    0, -20, JointType::symmetric, "rkneepitch", "lkneepitch");

    robot.connect_joint("lllo", "lean", .0, .0, +.5*radius_ankles                          , 'x', -45, +45, +5, JointType::normal,    "lanklepitch", "");
    robot.connect_joint("rllo", "rian", .0, .0, +.5*radius_ankles                          , 'x', -45, +45, +5, JointType::symmetric, "ranklepitch", "lanklepitch");
    robot.connect_joint("lean", "left", .0, 0.5*length_feet - radius_leg_lower, height_feet, 'y', -45, +45,   0, JointType::normal,    "lankleroll", "");
    robot.connect_joint("rian", "rift", .0, 0.5*length_feet - radius_leg_lower, height_feet, 'Y', -45, +45,   0, JointType::symmetric, "rankleroll", "lankleroll");

    /* attach sensors */
    robot.attach_accel_sensor("head");

    /* camera */
    robot.set_camera_center_on("upto");
    robot.setup_camera(Vector3(0.76, -0.78, .55), 135, -5, 0);

}

}} // namespace Robots::Biped

#endif // HUMANOID_H_INCLUDED
