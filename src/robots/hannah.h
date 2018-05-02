#ifndef HANNAH_H_INCLUDED
#define HANNAH_H_INCLUDED

#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Hannah {

/* body */
const double body_x = .250;
const double body_y = .500;
const double body_z = .120;

/* legs upper */
const double leg_upper_len = .32;
const double leg_upper_wx  = .01;
const double leg_upper_wy  = .06;

/* legs lower */
const double len_leg_lower = 0.40;
const double rad_leg_lower = 0.01;

/* shoulders */
const double shoulder_a = .04;

const double joint_distance = .005;
const double fx = 0.028;
const double fz = 0.016;

const double zheight_start = leg_upper_len + len_leg_lower + rad_leg_lower;// + joint_distance;


/* weights */
const double weight_body_kg      = 3.040; /**TODO*/
const double weight_shoulder_kg  = 0.100; /**TODO*/
const double weight_leg_upper_kg = 0.615; /**TODO*/


void
create_hannah(Robot& robot)
{
    dsPrint("Creating Hannah <3 \n");
    double xpos, ypos, zpos;

    /* body */
    xpos = .0;
    ypos = .0;
    zpos = zheight_start;
    robot.create_box("body", xpos, ypos, zpos, body_x, body_y, body_z, weight_body_kg, 0, colors::black, true, constants::friction::hi); // body

    /* shoulders */
    const double shoulder_z = zheight_start + 0.5*body_z - fx;

    zpos = shoulder_z;
    ypos = -.5*body_y -1.5*shoulder_a;
    xpos = 0.5*body_x - fx;

    robot.create_box("lfsh", +xpos, +ypos, zpos, shoulder_a, shoulder_a, shoulder_a, weight_shoulder_kg, 0, colors::black, true, constants::friction::lo); // left fore shoulder
    robot.create_box("rfsh", -xpos, +ypos, zpos, shoulder_a, shoulder_a, shoulder_a, weight_shoulder_kg, 0, colors::black, true, constants::friction::lo); // right fore shoulder
    robot.create_box("lhsh", +xpos, -ypos, zpos, shoulder_a, shoulder_a, shoulder_a, weight_shoulder_kg, 0, colors::black, true, constants::friction::lo); // left hind shoulder
    robot.create_box("rhsh", -xpos, -ypos, zpos, shoulder_a, shoulder_a, shoulder_a, weight_shoulder_kg, 0, colors::black, true, constants::friction::lo); // right hind shoulder

    /* legs upper */
    zpos = shoulder_z + fz - .5*leg_upper_len;
    xpos += 1.5*shoulder_a + 0.5*leg_upper_wx;

    robot.create_box("lflu", +xpos, +ypos, zpos, leg_upper_wx, leg_upper_wy, leg_upper_len, weight_leg_upper_kg, 0, colors::white, true, constants::friction::hi); // left fore leg upper
    robot.create_box("rflu", -xpos, +ypos, zpos, leg_upper_wx, leg_upper_wy, leg_upper_len, weight_leg_upper_kg, 0, colors::white, true, constants::friction::hi); // right fore leg upper
    robot.create_box("lhlu", +xpos, -ypos, zpos, leg_upper_wx, leg_upper_wy, leg_upper_len, weight_leg_upper_kg, 0, colors::white, true, constants::friction::hi); // left hind leg upper
    robot.create_box("rhlu", -xpos, -ypos, zpos, leg_upper_wx, leg_upper_wy, leg_upper_len, weight_leg_upper_kg, 0, colors::white, true, constants::friction::hi); // right hind leg upper

    /* legs lower */
    zpos = shoulder_z + fz - leg_upper_len - .5*len_leg_lower;

    const double dy = leg_upper_wy/2;
    const double D = dy+0.01;
    const double K = 0.034;

    robot.create_segment("lfll", +xpos, +ypos + dy, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::light, colors::black, true, constants::friction::sticky); // left fore leg lower
    robot.create_segment("rfll", -xpos, +ypos + dy, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::light, colors::black, true, constants::friction::sticky); // right fore leg lower
    robot.create_segment("lhll", +xpos, -ypos + dy, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::light, colors::black, true, constants::friction::sticky); // left hind leg lower
    robot.create_segment("rhll", -xpos, -ypos + dy, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::light, colors::black, true, constants::friction::sticky); // right hind leg lower

    /* connect by joints */
    robot.connect_joint("body", "lfsh", .0, .0, .0,                     'y', -90,  +90,  -5, JointType::normal,    "L_shoulder_roll"                     );
    robot.connect_joint("body", "rfsh", .0, .0, .0,                     'Y', -90,  +90,  -5, JointType::symmetric, "R_shoulder_roll" , "L_shoulder_roll" );
    robot.connect_joint("body", "lhsh", .0, .0, .0,                     'y', -90,  +90,  -5, JointType::normal,    "L_hip_roll"                          );
    robot.connect_joint("body", "rhsh", .0, .0, .0,                     'Y', -90,  +90,  -5, JointType::symmetric, "R_hip_roll"      , "L_hip_roll"      );


    robot.connect_joint("lfsh", "lflu", .0, .0, +.5*leg_upper_len - fz, 'x', -90,  +90, -15, JointType::normal,    "L_shoulder_pitch"                    );
    robot.connect_joint("rfsh", "rflu", .0, .0, +.5*leg_upper_len - fz, 'x', -90,  +90, -15, JointType::symmetric, "R_shoulder_pitch", "L_shoulder_pitch");
    robot.connect_joint("lflu", "lfll", .0, -D, +.5*len_leg_lower + K , 'x',   0, +180, +30, JointType::normal,    "L_elbow_pitch"                       );
    robot.connect_joint("rflu", "rfll", .0, -D, +.5*len_leg_lower + K , 'x',   0, +180, +30, JointType::symmetric, "R_elbow_pitch"   , "L_elbow_pitch"   );

    robot.connect_joint("lhsh", "lhlu", .0, .0, +.5*leg_upper_len - fz, 'x', -90,  +90, -25, JointType::normal,    "L_hip_pitch"                         );
    robot.connect_joint("rhsh", "rhlu", .0, .0, +.5*leg_upper_len - fz, 'x', -90,  +90, -25, JointType::symmetric, "R_hip_pitch"     , "L_hip_pitch"     );
    robot.connect_joint("lhlu", "lhll", .0, -D, +.5*len_leg_lower + K , 'x',   0, +180, +25, JointType::normal,    "L_knee_pitch"                        );
    robot.connect_joint("rhlu", "rhll", .0, -D, +.5*len_leg_lower + K , 'x',   0, +180, +25, JointType::symmetric, "R_knee_pitch"    , "L_knee_pitch"    );

    /* attach sensors */
    robot.attach_accel_sensor("body");

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(1.5, -0.5, 0.7), 140, -10, 0);

}


void
create_hannah_leg(Robot& robot)
{
    const Vector3 size_base (.5, .5, 1.25);

    dsPrint("Creating single leg of Hannah robot.\n");
    double xpos, ypos, zpos;

    /* body */
    xpos = .0;
    ypos = .05 + leg_upper_len + len_leg_lower;
    zpos = size_base.z/2 + 0.001;
    robot.create_box("body", xpos, ypos, zpos, body_x, body_y, size_base.z, 0, constants::materials::rock, colors::black_t, true, constants::friction::hi); // body

    /* shoulders */
    const double shoulder_z = size_base.z - fx;

    zpos = shoulder_z;
    ypos = -.5*body_y -1.5*shoulder_a;
    xpos = 0.5*body_x - fx;

    robot.create_box("lfsh", +xpos, +ypos, zpos, shoulder_a, shoulder_a, shoulder_a, 0, constants::materials::light, colors::black, true, constants::friction::lo); // left fore shoulder

    /* legs upper */
    zpos = shoulder_z + fz - .5*leg_upper_len;
    xpos += 1.5*shoulder_a + 0.5*leg_upper_wx;

    robot.create_box("lflu", +xpos, +ypos, zpos, leg_upper_wx, leg_upper_wy, leg_upper_len, 0, constants::materials::body, colors::white, true, constants::friction::hi); // left fore leg upper

    /* legs lower */
    zpos = shoulder_z + fz - leg_upper_len - .5*len_leg_lower;

    const double dy = leg_upper_wy/2;
    const double D = dy+0.01;
    const double K = 0.034;

    robot.create_segment("lfll", +xpos, +ypos + dy, zpos, 3, len_leg_lower, rad_leg_lower, 0, constants::materials::light, colors::black, true, constants::friction::hi); // left fore leg lower

    /* connect by joints */
    robot.connect_joint("body", "lfsh", .0, .0, .0,                     'y', -90,  +90,  +5, JointType::normal, "L_shoulder_roll"  );
    robot.connect_joint("lfsh", "lflu", .0, .0, +.5*leg_upper_len - fz, 'x', -90,  +90, -10, JointType::normal, "L_shoulder_pitch" );
    robot.connect_joint("lflu", "lfll", .0, -D, +.5*len_leg_lower + K , 'x',   0, +180, +20, JointType::normal, "L_elbow_pitch"    );

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(2.0, -1.0, 1.45), 140, -17, 0);
}

}} // namespace Robots::Hannah

#endif // HANNAH_H_INCLUDED
