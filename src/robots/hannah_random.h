#ifndef HANNAH_RANDOM_H_INCLUDED
#define HANNAH_RANDOM_H_INCLUDED

#include <basic/common.h>
#include <build/robot.h>
#include <build/physics.h>



namespace Robots {
namespace HannahRand {

    inline double rnd(double m, double s, double a)
    {
        assert(0.0 <= s and s <= 1.0);
        assert(0.0 <= a and a <= 1.0);
        const float rmin = (1 - a*s) * m;
        const float rmax = (1 + a*s) * m;

        assert(rmin >= 0. and rmax <= 2*m);
        return common::getRandomDouble(rmin, rmax);

    }

    struct Capsule {
        Capsule(double l, double r) : len(l), rad(r) {}

        double len, rad; };

    struct HannahMorphology
    {
        Vector3 body;
        Vector3 leg_upper;
        Capsule leg_lower;

        double shoulder_a;

        const double fx = 0.028; //TODO rename
        const double fz = 0.016;

        double zheight_start;


        /* weights */
        struct Weights {
            double body;
            double shoulder;
            double leg_upper;
            double leg_lower;
        } weight_kg;


        HannahMorphology(double amp)
        : body( rnd(.250, 0.2, amp)
              , rnd(.500, 0.2, amp)
              , rnd(.120, 0.2, amp)
              )
        , leg_upper( rnd(.010, 0.2, amp)
                   , rnd(.060, 0.2, amp)
                   , rnd(.320, 0.2, amp) // length
                   )
        , leg_lower( rnd(0.50, 0.2, amp) // length
                   , rnd(0.01, 0.2, amp) // radius
                   )
        , shoulder_a( rnd(.04, 0.2, amp) )
        , zheight_start( leg_upper.z + leg_lower.len + leg_lower.rad )
        , weight_kg({rnd(3.040, 0.2, amp)  /**TODO specify */
                   , rnd(0.100, 0.2, amp)  /**TODO specify */
                   , rnd(0.615, 0.2, amp)  /**TODO specify */
                   , rnd(0.200, 0.2, amp)  /**TODO specify */
                   })
        {
            dsPrint("Body: %1.2f %1.2f %1.2f", body.x, body.y, body.z);
        }

    };

void
create_random_hannah(Robot& robot, unsigned rnd_instance, double rnd_amp)
{

    if (rnd_instance != 0)
        srand(rnd_instance);

    HannahMorphology m(rnd_amp);

    srand(time(NULL)); // usual seed for random number generator.

    dsPrint("Creating randomized Hannah <3 \n");

    /* body */
    Vector3 pos (.0, .0, m.zheight_start);
    robot.create_box("body", pos, m.body, m.weight_kg.body, 0, colors::black, true, constants::friction::hi); // body

    /* shoulders */
    const double shoulder_z = m.zheight_start + 0.5*m.body.z - m.fx;

    pos.z = shoulder_z;
    pos.y = -.5*m.body.y -1.5*m.shoulder_a;
    pos.x = 0.5*m.body.x - m.fx;

    robot.create_box("lfsh", {+pos.x, +pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, colors::black, true, constants::friction::lo); // left fore shoulder
    robot.create_box("rfsh", {-pos.x, +pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, colors::black, true, constants::friction::lo); // right fore shoulder
    robot.create_box("lhsh", {+pos.x, -pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, colors::black, true, constants::friction::lo); // left hind shoulder
    robot.create_box("rhsh", {-pos.x, -pos.y, pos.z}, m.shoulder_a, m.weight_kg.shoulder, 0, colors::black, true, constants::friction::lo); // right hind shoulder

    /* legs upper */
    pos.z = shoulder_z + m.fz - .5*m.leg_upper.z;
    pos.x += 1.5*m.shoulder_a + 0.5*m.leg_upper.x;

    robot.create_box("lflu", {+pos.x, +pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, colors::white, true, constants::friction::hi); // left fore leg upper
    robot.create_box("rflu", {-pos.x, +pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, colors::white, true, constants::friction::hi); // right fore leg upper
    robot.create_box("lhlu", {+pos.x, -pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, colors::white, true, constants::friction::hi); // left hind leg upper
    robot.create_box("rhlu", {-pos.x, -pos.y, pos.z}, m.leg_upper, m.weight_kg.leg_upper, 0, colors::white, true, constants::friction::hi); // right hind leg upper

    /* legs lower */
    pos.z = shoulder_z + m.fz - m.leg_upper.z - .5*m.leg_lower.len;

    const double dy = m.leg_upper.y/2;
    const double D = dy+0.01;
    const double K = 0.034;

    robot.create_segment("lfll", +pos.x, +pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, colors::black, true, constants::friction::sticky); // left fore leg lower
    robot.create_segment("rfll", -pos.x, +pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, colors::black, true, constants::friction::sticky); // right fore leg lower
    robot.create_segment("lhll", +pos.x, -pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, colors::black, true, constants::friction::sticky); // left hind leg lower
    robot.create_segment("rhll", -pos.x, -pos.y + dy, pos.z, 3, m.leg_lower.len, m.leg_lower.rad, m.weight_kg.leg_lower, 0, colors::black, true, constants::friction::sticky); // right hind leg lower

    /* connect by joints */
    robot.connect_joint("body", "lfsh", .0, .0, .0,                       'y', -90,  +90,  -1, JointType::normal,    "L_shoulder_roll"                     );
    robot.connect_joint("body", "rfsh", .0, .0, .0,                       'Y', -90,  +90,  -1, JointType::symmetric, "R_shoulder_roll" , "L_shoulder_roll" );
    robot.connect_joint("body", "lhsh", .0, .0, .0,                       'y', -90,  +90,  -1, JointType::normal,    "L_hip_roll"                          );
    robot.connect_joint("body", "rhsh", .0, .0, .0,                       'Y', -90,  +90,  -1, JointType::symmetric, "R_hip_roll"      , "L_hip_roll"      );


    robot.connect_joint("lfsh", "lflu", .0, .0, +.5*m.leg_upper.z - m.fz, 'x', -90,  +90, -12, JointType::normal,    "L_shoulder_pitch"                    );
    robot.connect_joint("rfsh", "rflu", .0, .0, +.5*m.leg_upper.z - m.fz, 'x', -90,  +90, -12, JointType::symmetric, "R_shoulder_pitch", "L_shoulder_pitch");
    robot.connect_joint("lflu", "lfll", .0, -D, +.5*m.leg_lower.len + K , 'x',   0, +180, +30, JointType::normal,    "L_elbow_pitch"                       );
    robot.connect_joint("rflu", "rfll", .0, -D, +.5*m.leg_lower.len + K , 'x',   0, +180, +30, JointType::symmetric, "R_elbow_pitch"   , "L_elbow_pitch"   );

    robot.connect_joint("lhsh", "lhlu", .0, .0, +.5*m.leg_upper.z - m.fz, 'x', -90,  +90, -15, JointType::normal,    "L_hip_pitch"                         );
    robot.connect_joint("rhsh", "rhlu", .0, .0, +.5*m.leg_upper.z - m.fz, 'x', -90,  +90, -15, JointType::symmetric, "R_hip_pitch"     , "L_hip_pitch"     );
    robot.connect_joint("lhlu", "lhll", .0, -D, +.5*m.leg_lower.len + K , 'x',   0, +180, +15, JointType::normal,    "L_knee_pitch"                        );
    robot.connect_joint("rhlu", "rhll", .0, -D, +.5*m.leg_lower.len + K , 'x',   0, +180, +15, JointType::symmetric, "R_knee_pitch"    , "L_knee_pitch"    );

    /* attach sensors */
    robot.attach_accel_sensor("body");

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(1.5, -0.5, 0.7), 140, -10, 0);
}


}} // namespace Robots::Hannah

#endif // HANNAH_RANDOM_H_INCLUDED
