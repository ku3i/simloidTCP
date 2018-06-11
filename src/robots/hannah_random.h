#ifndef HANNAH_RANDOM_H_INCLUDED
#define HANNAH_RANDOM_H_INCLUDED

#include <basic/common.h>
#include <basic/color.h>
#include <build/robot.h>
#include <build/params.h>
#include <build/physics.h>


namespace Robots {
namespace HannahRand {

    using common::rnd;

    const double range = 0.2;

    struct Capsule {
        Capsule(double l, double r) : len(l), rad(r) {}

        double len, rad; };

    struct HannahMorphology
    {
        const Vector3 body;
        const Vector3 leg_upper;
        const Capsule leg_lower;

        const double shoulder_a;

        const double body_shld_joint_dist_xz;
        const double shld_legs_joint_dist_z;

        const double zheight_start;

        /* TODO: consider increasing the range here */
        const double ground_contact_friction;

        /* weights */
        const struct Weights {
            double body;
            double shoulder;
            double leg_upper;
            double leg_lower;
        } weight_kg;

        const double torque;

        Color4 color_body;
        Color4 color_dark;
        Color4 color_light;

        HannahMorphology(double amp)
        : body( rnd(.250, range, amp)
              , rnd(.500, range, amp)
              , rnd(.120, range, amp)
              )
        , leg_upper( rnd(.010, range, amp)
                   , rnd(.060, range, amp)
                   , rnd(.320, range, amp) // length
                   )
        , leg_lower( rnd(0.50, range, amp) // length
                   , rnd(0.01, range, amp) // radius
                   )
        , shoulder_a( rnd(.04, range, amp) )
        , body_shld_joint_dist_xz( rnd(.028, range, amp) )
        , shld_legs_joint_dist_z ( rnd(.016, range, amp) )
        , zheight_start( leg_upper.z + leg_lower.len + leg_lower.rad )
        , ground_contact_friction( rnd(constants::friction::sticky, range, amp) )
        , weight_kg({rnd(3.040, range, amp)  /**TODO specify */
                   , rnd(0.100, range, amp)  /**TODO specify */
                   , rnd(0.615, range, amp)  /**TODO specify */
                   , rnd(0.200, range, amp)  /**TODO specify, make the real leg weight more, in order to back-drive */
                   })
        , torque( rnd(5.0, range, amp) )
        , color_body ( rnd(.5, 1.0, amp), rnd(.5, 1.0, amp), rnd(.5, 1.0, amp), 1.0 )
        , color_dark ( rnd(.3, .50, amp), rnd(.3, .50, amp), rnd(.3, .50, amp), 1.0 )
        , color_light( rnd(.9, .10, amp), rnd(.9, .10, amp), rnd(.9, .10, amp), 1.0 )
        {
            dsPrint("Model Parameters:\n"
                    "\tBody       : m=%1.4f l=(%1.4f %1.4f %1.4f)\n"
                    "\tShoulder   : m=%1.4f l=(%1.4f)\n"
                    "\tLegs Upper : m=%1.4f l=(%1.4f %1.4f %1.4f)\n"
                    "\tLegs Lower : m=%1.4f l=(%1.4f %1.4f)\n"
                    , weight_kg.body     , body.x       , body.y       , body.z
                    , weight_kg.shoulder , shoulder_a
                    , weight_kg.leg_upper, leg_upper.x  , leg_upper.y  , leg_upper.z
                    , weight_kg.leg_lower, leg_lower.len, leg_lower.rad
                    );
        }

    };

void
create_random_hannah(Robot& robot, std::vector<double> model_parameter)
{
    unsigned rnd_instance = 0;
    double   rnd_amp = .0;

    if (model_parameter.size() == 2) {
        rnd_instance = static_cast<unsigned>(model_parameter[0]);
        rnd_amp      = model_parameter[1];
        dsPrint("Using model parameters instance %u and random amplitude %lf", rnd_instance, rnd_amp);
    }
    else if (model_parameter.size() == 0)
        dsPrint("No model parameters provided, taking defaults.");
    else
        dsError("Wrong number of model parameters.");

    if (rnd_instance != 0)
        srand(rnd_instance);

    HannahMorphology m(rnd_amp);
    ActuatorParameters params(range, rnd_amp);

    srand(time(NULL)); // usual seed for random number generator.

    dsPrint("Creating randomized Hannah <3\n");

    const double X = 3*rnd_amp;

    /* body */
    Vector3 pos (.0, .0, m.zheight_start);
    robot.create_box("body", pos, m.body, m.weight_kg.body, 0, m.color_body, true, constants::friction::hi); // body

    /* shoulders */
    const double shoulder_z = m.zheight_start + 0.5*m.body.z - m.body_shld_joint_dist_xz;

    pos.z = shoulder_z;
    pos.y = -.5*m.body.y -1.5*m.shoulder_a;
    pos.x = 0.5*m.body.x - m.body_shld_joint_dist_xz;

    robot.create_box("lfsh", {+pos.x, +pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, m.color_dark, true, constants::friction::lo); // left fore shoulder
    robot.create_box("rfsh", {-pos.x, +pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, m.color_dark, true, constants::friction::lo); // right fore shoulder
    robot.create_box("lhsh", {+pos.x, -pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, m.color_dark, true, constants::friction::lo); // left hind shoulder
    robot.create_box("rhsh", {-pos.x, -pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, m.color_dark, true, constants::friction::lo); // right hind shoulder

    /* legs upper */
    pos.z = shoulder_z + m.shld_legs_joint_dist_z - .5*m.leg_upper.z;
    pos.x += 1.5*m.shoulder_a + 0.5*m.leg_upper.x;

    robot.create_box("lflu", {+pos.x, +pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi); // left fore leg upper
    robot.create_box("rflu", {-pos.x, +pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi); // right fore leg upper
    robot.create_box("lhlu", {+pos.x, -pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi); // left hind leg upper
    robot.create_box("rhlu", {-pos.x, -pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi); // right hind leg upper

    /* legs lower */
    pos.z = shoulder_z + m.shld_legs_joint_dist_z - m.leg_upper.z - .5*m.leg_lower.len;

    const double dy = m.leg_upper.y/2;
    const double D = dy+0.01;
    const double K = 0.034;

    /* relative joint positions */
    const Vector3 jpu = {.0, .0, +.5*m.leg_upper.z - m.shld_legs_joint_dist_z};
    const Vector3 jpl = {.0, -D, +.5*m.leg_lower.len + K };

    robot.create_segment("lfll", +pos.x, +pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, m.color_dark, true, m.ground_contact_friction); // left fore leg lower
    robot.create_segment("rfll", -pos.x, +pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, m.color_dark, true, m.ground_contact_friction); // right fore leg lower
    robot.create_segment("lhll", +pos.x, -pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, m.color_dark, true, m.ground_contact_friction); // left hind leg lower
    robot.create_segment("rhll", -pos.x, -pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, m.color_dark, true, m.ground_contact_friction); // right hind leg lower

    /* connect by joints */
    robot.connect_joint("body", "lfsh", .0, .0, .0,          'y', -90,  +90,  -1 + rnd(X), JointType::normal,    "L_shoulder_roll" , ""                , m.torque, params);
    robot.connect_joint("body", "rfsh", .0, .0, .0,          'Y', -90,  +90,  -1 + rnd(X), JointType::symmetric, "R_shoulder_roll" , "L_shoulder_roll" , m.torque, params);
    robot.connect_joint("body", "lhsh", .0, .0, .0,          'y', -90,  +90,  -1 + rnd(X), JointType::normal,    "L_hip_roll"      , ""                , m.torque, params);
    robot.connect_joint("body", "rhsh", .0, .0, .0,          'Y', -90,  +90,  -1 + rnd(X), JointType::symmetric, "R_hip_roll"      , "L_hip_roll"      , m.torque, params);

    robot.connect_joint("lfsh", "lflu", jpu.x, jpu.y, jpu.z, 'x', -90,  +90, -12 + rnd(X), JointType::normal,    "L_shoulder_pitch", ""                , m.torque, params);
    robot.connect_joint("rfsh", "rflu", jpu.x, jpu.y, jpu.z, 'x', -90,  +90, -12 + rnd(X), JointType::symmetric, "R_shoulder_pitch", "L_shoulder_pitch", m.torque, params);
    robot.connect_joint("lflu", "lfll", jpl.x, jpl.y, jpl.z, 'x',   0, +180, +30 + rnd(X), JointType::normal,    "L_elbow_pitch"   , ""                , m.torque, params);
    robot.connect_joint("rflu", "rfll", jpl.x, jpl.y, jpl.z, 'x',   0, +180, +30 + rnd(X), JointType::symmetric, "R_elbow_pitch"   , "L_elbow_pitch"   , m.torque, params);

    robot.connect_joint("lhsh", "lhlu", jpu.x, jpu.y, jpu.z, 'x', -90,  +90, -15 + rnd(X), JointType::normal,    "L_hip_pitch"     , ""                , m.torque, params);
    robot.connect_joint("rhsh", "rhlu", jpu.x, jpu.y, jpu.z, 'x', -90,  +90, -15 + rnd(X), JointType::symmetric, "R_hip_pitch"     , "L_hip_pitch"     , m.torque, params);
    robot.connect_joint("lhlu", "lhll", jpl.x, jpl.y, jpl.z, 'x',   0, +180, +15 + rnd(X), JointType::normal,    "L_knee_pitch"    , ""                , m.torque, params);
    robot.connect_joint("rhlu", "rhll", jpl.x, jpl.y, jpl.z, 'x',   0, +180, +15 + rnd(X), JointType::symmetric, "R_knee_pitch"    , "L_knee_pitch"    , m.torque, params);

    /* attach sensors */
    robot.attach_accel_sensor("body", /* keep original color = */true);

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(1.5, -0.5, 0.7), 140, -10, 0);
}


}} // namespace Robots::Hannah

#endif // HANNAH_RANDOM_H_INCLUDED
