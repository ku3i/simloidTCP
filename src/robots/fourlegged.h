#ifndef FOURLEGGED_H_INCLUDED
#define FOURLEGGED_H_INCLUDED

#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Fourlegged {

/* body */
const double len_body = .300;
const double rad_body = .05;

/* legs upper */
const double rad_leg_upper = .66*rad_body;
const double len_leg_upper = .50*len_body - rad_leg_upper;

/* legs lower */
const double len_leg_lower = 1.33*len_leg_upper;
const double rad_leg_lower = 0.80*rad_leg_upper;

/* shoulders */
const double len_shoulder = .075;
const double rad_shoulder = .015;

const double joint_distance = .005;

const double zheight_start = len_leg_upper + len_leg_lower + rad_leg_lower + joint_distance;//.36;

void
create_wildcat_0(Robot& robot)
{
    dsPrint("Wildcat_0 (four-legged)\n");
    double xpos, ypos, zpos;

    /* body */
    xpos = .000;
    ypos = rad_body + .5*len_body;
    zpos = zheight_start;
    robot.create_segment("body", xpos, ypos, zpos, 2, len_body, rad_body, 0, constants::materials::light, colors::black, true, constants::friction::hi); // body

    /* shoulders */
    zpos = zheight_start;
    ypos = rad_body;
    xpos = rad_body + rad_leg_upper + joint_distance;
    robot.create_segment("lfsh", +xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::black, true, constants::friction::lo); // left fore shoulder
    robot.create_segment("rfsh", -xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::black, true, constants::friction::lo); // right fore shoulder

    ypos = rad_body + len_body;
    robot.create_segment("lhsh", +xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::black, true, constants::friction::lo); // left hind shoulder
    robot.create_segment("rhsh", -xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::black, true, constants::friction::lo); // right hind shoulder

    /* fore legs upper */
    zpos = zheight_start - .5*len_leg_upper;
    ypos = rad_body;
    robot.create_segment("lflu", +xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // left fore leg upper
    robot.create_segment("rflu", -xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // right fore leg upper

    /* fore legs lower */
    zpos = zheight_start - len_leg_upper - .5*len_leg_lower;
    robot.create_segment("lfll", +xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // left fore leg lower
    robot.create_segment("rfll", -xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // right fore leg lower

    /* hind legs upper */
    zpos = zheight_start - .5*len_leg_upper;
    ypos = rad_body + len_body;
    robot.create_segment("lhlu", +xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // left hind leg upper
    robot.create_segment("rhlu", -xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // right hind leg upper

    /* hind legs lower */
    zpos = zheight_start - len_leg_upper - .5*len_leg_lower;
    robot.create_segment("lhll", +xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // left hind leg lower
    robot.create_segment("rhll", -xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // right hind leg lower

    /* connect by joints */
    robot.connect_joint("body", "lfsh", .0, .0, .0,                'x', -90,  +90, -30, JointType::normal,    "L_shoulder_pitch"                    );
    robot.connect_joint("body", "rfsh", .0, .0, .0,                'x', -90,  +90, -30, JointType::symmetric, "R_shoulder_pitch", "L_shoulder_pitch");
    robot.connect_joint("body", "lhsh", .0, .0, .0,                'x', -90,  +90, -60, JointType::normal,    "L_hip_pitch"                         );
    robot.connect_joint("body", "rhsh", .0, .0, .0,                'x', -90,  +90, -60, JointType::symmetric, "R_hip_pitch"     , "L_hip_pitch"     );


    robot.connect_joint("lfsh", "lflu", .0, .0, +.5*len_leg_upper, 'y', -90,  +90, +10, JointType::normal,    "L_shoulder_roll"                     );
    robot.connect_joint("rfsh", "rflu", .0, .0, +.5*len_leg_upper, 'Y', -90,  +90, +10, JointType::symmetric, "R_shoulder_roll" , "L_shoulder_roll" );
    robot.connect_joint("lflu", "lfll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::normal,    "L_elbow_pitch"                       );
    robot.connect_joint("rflu", "rfll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::symmetric, "R_elbow_pitch"   , "L_elbow_pitch"   );

    robot.connect_joint("lhsh", "lhlu", .0, .0, +.5*len_leg_upper, 'y', -90,  +90, +10, JointType::normal,    "L_hip_roll"                          );
    robot.connect_joint("rhsh", "rhlu", .0, .0, +.5*len_leg_upper, 'Y', -90,  +90, +10, JointType::symmetric, "R_hip_roll"      , "L_hip_roll"      );
    robot.connect_joint("lhlu", "lhll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::normal,    "L_knee_pitch"                        );
    robot.connect_joint("rhlu", "rhll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::symmetric, "R_knee_pitch"    , "L_knee_pitch"    );

    /* attach sensors */
    robot.attach_accel_sensor("body");

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(0.9, -0.7, .4), 120, 10, 0);

}

void
create_wildcat_1(Robot& robot) /* alternative roll joints */
{
    dsPrint("Wildcat_1 (four-legged)\n");
    double xpos, ypos, zpos;

    /* body */
    xpos = .000;
    ypos = rad_body + .5*len_body;
    zpos = zheight_start;
    robot.create_segment("body", xpos, ypos, zpos, 2, len_body, rad_body, 0, constants::materials::light, colors::black, true, constants::friction::hi); // body

    /* shoulders */
    zpos = zheight_start;
    ypos = rad_body;
    xpos = rad_body + rad_leg_upper + joint_distance;
    robot.create_segment("lfsh", +xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::invisible, true, constants::friction::lo); // left fore shoulder
    robot.create_segment("rfsh", -xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::invisible, true, constants::friction::lo); // right fore shoulder

    ypos = rad_body + len_body;
    robot.create_segment("lhsh", +xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::invisible, true, constants::friction::lo); // left hind shoulder
    robot.create_segment("rhsh", -xpos, ypos, zpos, 2, len_shoulder, rad_shoulder,   0, constants::materials::body, colors::invisible, true, constants::friction::lo); // right hind shoulder

    /* fore legs upper */
    zpos = zheight_start - .5*len_leg_upper;
    ypos = rad_body;
    robot.create_segment("lflu", +xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // left fore leg upper
    robot.create_segment("rflu", -xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // right fore leg upper

    /* fore legs lower */
    zpos = zheight_start - len_leg_upper - .5*len_leg_lower;
    robot.create_segment("lfll", +xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // left fore leg lower
    robot.create_segment("rfll", -xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // right fore leg lower

    /* hind legs upper */
    zpos = zheight_start - .5*len_leg_upper;
    ypos = rad_body + len_body;
    robot.create_segment("lhlu", +xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // left hind leg upper
    robot.create_segment("rhlu", -xpos, ypos, zpos, 3, len_leg_upper, rad_leg_upper, 0, constants::materials::body, colors::white, true, constants::friction::hi); // right hind leg upper

    /* hind legs lower */
    zpos = zheight_start - len_leg_upper - .5*len_leg_lower;
    robot.create_segment("lhll", +xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // left hind leg lower
    robot.create_segment("rhll", -xpos, ypos, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::body, colors::black, true, constants::friction::hi); // right hind leg lower

    /* connect by joints */
    robot.connect_joint("body", "lfsh", .0, .0, .0,                'y', -90,  +90, +10, JointType::normal,    "L_shoulder_roll"                     );
    robot.connect_joint("body", "rfsh", .0, .0, .0,                'Y', -90,  +90, +10, JointType::symmetric, "R_shoulder_roll" , "L_shoulder_roll" );
    robot.connect_joint("body", "lhsh", .0, .0, .0,                'y', -90,  +90, +10, JointType::normal,    "L_hip_roll"                          );
    robot.connect_joint("body", "rhsh", .0, .0, .0,                'Y', -90,  +90, +10, JointType::symmetric, "R_hip_roll"      , "L_hip_roll"      );


    robot.connect_joint("lfsh", "lflu", .0, .0, +.5*len_leg_upper, 'x', -90,  +90, -30, JointType::normal,    "L_shoulder_pitch"                    );
    robot.connect_joint("rfsh", "rflu", .0, .0, +.5*len_leg_upper, 'x', -90,  +90, -30, JointType::symmetric, "R_shoulder_pitch", "L_shoulder_pitch");
    robot.connect_joint("lflu", "lfll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::normal,    "L_elbow_pitch"                       );
    robot.connect_joint("rflu", "rfll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::symmetric, "R_elbow_pitch"   , "L_elbow_pitch"   );

    robot.connect_joint("lhsh", "lhlu", .0, .0, +.5*len_leg_upper, 'x', -90,  +90, -60, JointType::normal,    "L_hip_pitch"                         );
    robot.connect_joint("rhsh", "rhlu", .0, .0, +.5*len_leg_upper, 'x', -90,  +90, -60, JointType::symmetric, "R_hip_pitch"     , "L_hip_pitch"     );
    robot.connect_joint("lhlu", "lhll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::normal,    "L_knee_pitch"                        );
    robot.connect_joint("rhlu", "rhll", .0, .0, +.5*len_leg_lower, 'x',   0, +120, +60, JointType::symmetric, "R_knee_pitch"    , "L_knee_pitch"    );

    /* attach sensors */
    robot.attach_accel_sensor("body");

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(0.9, -0.7, .4), 120, 10, 0);

}

}} // namespace Robots::Fourlegged

#endif // FOURLEGGED_H_INCLUDED
