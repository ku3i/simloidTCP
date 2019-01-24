#ifndef HANNAH_DETAIL_RND_H_INCLUDED
#define HANNAH_DETAIL_RND_H_INCLUDED

#include <basic/common.h>
#include <basic/color.h>
#include <basic/capsule.h>
#include <build/robot.h>
#include <build/params.h>
#include <build/physics.h>


namespace Robots {
namespace HannahRandDetail {

    using common::rnd;

    const double range = 0.2;

    const struct MotorDimensions { //TODO randomize?
        double width  = 0.030;
        double height = 0.065;
        double depth  = 0.075;
        double offset = 0.010;
    } mot;

    const Vector3 motor_face_front = {mot.width , mot.depth , mot.height};
    const Vector3 motor_face_side  = {mot.depth , mot.width , mot.height};

    /*

    bristle_displ_max :  0.0511 | bristle_stiffness :  0.8863
    sticking_friction :  0.2471 | coulomb_friction  :  0.1991
    fluid_friction    :  0.1209 | stiction_range    :  0.0515
    V_in              : 12.0000 | kB                :  2.0203
    kM                :  2.0356 | R_i_inv           :  0.0727


    */

    ActuatorParameters Sensorimotor {{/* bristle_displ_max = */ +1.00000000e-03,
                                      /* bristle_stiffness = */ +1.00348110e+00,
                                      /* sticking_friction = */ +2.74093907e-01,
                                      /* coulomb_friction  = */ +2.02936051e-01,
                                      /* fluid_friction    = */ +1.81613953e-01,
                                      /* stiction_range    = */ +3.39670766e-02,
                                      /* V_in              = */ +1.20000000e+01,
                                      /* kB                = */ +1.95730358e+00,
                                      /* kM                = */ +1.93830452e+00,
                                      /* R_i_inv           = */ +8.19456429e-02
                                      }, /*assert_range=*/true };

    struct HannahMorphology
    {
        const Vector3 body;
        const Vector3 leg_upper;
        const Capsule leg_lower;
        const Vector3 knee;

        const double dbl_bearing;
        const double shoulder_a;

        const double body_shld_joint_dist_xz;
        const double shld_legs_joint_dist_z;
        const double knee_y_offset;

        const double zheight_start;

        /* TODO: consider increasing the range here */
        const double ground_contact_friction;

        /* weights */
        const struct Weights {
            double body;
            double shoulder;
            double leg_upper;
            double leg_lower;
            double knee;
            double motor;
            double dbl_bearing;
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
        , leg_lower( 3
                   , rnd(0.40, range, amp) // length
                   , rnd(0.01, range, amp) // radius
                   )
        , knee( rnd(0.02, range, amp)
              , rnd(0.04, range, amp)
              , rnd(0.04, range, amp)
              )
        , dbl_bearing( rnd(.030, range, amp))
        , shoulder_a( rnd(.04, range, amp) )
        , body_shld_joint_dist_xz( rnd(.028, range, amp) )
        , shld_legs_joint_dist_z ( rnd(.016, range, amp) )
        , knee_y_offset( rnd(.034, range, amp) )
        , zheight_start( leg_upper.z + leg_lower.len + leg_lower.rad )
        , ground_contact_friction( rnd(constants::friction::sticky, range, amp) )
        , weight_kg({rnd(1.000, range, amp) // body TODO specify
                   , rnd(0.030, range, amp) // shoulder TODO specify
                   , rnd(0.055, range, amp) // leg_upper TODO specify
                   , rnd(0.100, range, amp) // leg_lower TODO specify, make the real leg weight more and the leg shorter, in order to back-drive */
                   , rnd(0.050, range, amp) // knee TODO
                   , rnd(0.195, range, amp) // motor, incl. screws+nuts
                   , rnd(0.100, range, amp) // dbl_bearing with axis
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


void create_leg(Robot& robot, HannahMorphology const& m, Vector3 pos, std::string const& name, int sign)
{
    Vector3 pos_shoulder = pos;

    pos += {/*x*/ sign*(1.5*m.shoulder_a + .5*m.leg_upper.x)
           ,/*y*/ .0
           ,/*z*/ m.shld_legs_joint_dist_z - .5*m.leg_upper.z };

    Vector3 pos_leg_upper = pos;

    pos.y += .5*m.leg_upper.y - m.leg_lower.rad;
    pos.z += -.5*m.leg_upper.z - .5*m.leg_lower.len - m.leg_lower.rad;

    Vector3 pos_leg_lower = pos;

    /* shoulder  */ robot.create_box    (name + "sh", pos_shoulder , m.shoulder_a, m.weight_kg.shoulder , 0, m.color_dark , false);
    /* leg upper */ robot.create_box    (name + "lu", pos_leg_upper, m.leg_upper , m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi);
    /* leg lower */ robot.create_segment(name + "ll", pos_leg_lower, m.leg_lower , m.weight_kg.leg_lower, 0, m.color_dark , true, m.ground_contact_friction);

    /* attach motors */
    robot.attach_box(name+"lu", pos_leg_upper + Vector3{-sign*mot.offset,.0,+.04/**TODO*/}, motor_face_side, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box(name+"lu", pos_leg_upper + Vector3{-sign*mot.offset,.0,-.04/**TODO*/}, motor_face_side, m.weight_kg.motor, 0, colors::black_t, false);

    const Vector3 upper_bearing_pos = pos_leg_upper + Vector3{.0, .0, +.5*m.leg_upper.z - m.shld_legs_joint_dist_z};
    const Vector3 lower_bearing_pos = pos_leg_upper + Vector3{.0, -.25*m.leg_upper.y + .5*m.leg_lower.rad, -.5*m.leg_upper.z + m.knee_y_offset -m.leg_lower.rad };

    /* attach bearings */
    robot.attach_box(name+"lu", upper_bearing_pos, m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);
    robot.attach_box(name+"lu", lower_bearing_pos, m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);

    const Vector3 knee_pos = pos_leg_lower + Vector3{ .0, -.01, .5*m.leg_lower.len + 0.02};
    /* attach knee */
    robot.attach_box(name+"ll", knee_pos, m.knee, m.weight_kg.knee, 0, colors::magenta_t, false);
}

void
create_hannah_detail_random(Robot& robot, std::vector<double> model_parameter)
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
    ActuatorParameters params = Sensorimotor;
    params.randomize(range, rnd_amp);

    srand(time(NULL)); // reset usual seed for random number generator.

    dsPrint("Creating randomized Hannah <3\n");

    const double X = 3*rnd_amp;

    /* body */
    Vector3 body_pos (.0, .0, m.zheight_start);
    robot.create_box("body", body_pos, m.body, m.weight_kg.body, 0, m.color_body, true, constants::friction::hi); // body

    /* attach motors */
    const Vector3 pos_motor = body_pos + /**TODO*/Vector3{-.125*m.body.x, -.5*m.body.y + mot.offset, .0};
    robot.attach_box("body", pos_motor          , motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box("body", pos_motor._x()     , motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box("body", pos_motor     ._y(), motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box("body", pos_motor._x()._y(), motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);

    /* attach bearings */
    const Vector3 bearing_pos = body_pos + /**TODO*/Vector3{ .5*m.body.x - m.body_shld_joint_dist_xz
                                                           ,-.5*m.body.y /*- 1.5*m.shoulder_a*/
                                                           , .5*m.body.z - m.body_shld_joint_dist_xz
                                                           };

    robot.attach_box("body", bearing_pos          , m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);
    robot.attach_box("body", bearing_pos._x()     , m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);
    robot.attach_box("body", bearing_pos     ._y(), m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);
    robot.attach_box("body", bearing_pos._x()._y(), m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);


    const Vector3 shoulder =
    { 0.5*m.body.x - m.body_shld_joint_dist_xz
    , -.5*m.body.y - 1.5*m.shoulder_a
    , m.zheight_start + 0.5*m.body.z - m.body_shld_joint_dist_xz
    };

    create_leg(robot, m, {+shoulder.x, +shoulder.y, shoulder.z}, "lf",  1 );
    create_leg(robot, m, {-shoulder.x, +shoulder.y, shoulder.z}, "rf", -1 );
    create_leg(robot, m, {+shoulder.x, -shoulder.y, shoulder.z}, "lh",  1 );
    create_leg(robot, m, {-shoulder.x, -shoulder.y, shoulder.z}, "rh", -1 );


    /* relative joint positions */
    const Vector3 jpu = {.0, .0               , +.5*m.leg_upper.z - m.shld_legs_joint_dist_z};
    const Vector3 jpl = {.0, -.5*m.leg_upper.y, +.5*m.leg_lower.len + m.knee_y_offset };


    /* connect by joints */

    /*fore legs*/
    /*0*/ robot.connect_joint("body", "rfsh", 0.0, 'Y', -90,  +90, + 1 + rnd(X), JointType::normal   , "R_shoulder_roll" , ""                , m.torque, params);
    /*1*/ robot.connect_joint("body", "lfsh", 0.0, 'y', -90,  +90, + 1 + rnd(X), JointType::symmetric, "L_shoulder_roll" , "R_shoulder_roll" , m.torque, params);
    /*2*/ robot.connect_joint("rfsh", "rflu", jpu, 'x', -90,  +90, -15 + rnd(X), JointType::normal   , "R_shoulder_pitch", ""                , m.torque, params);
    /*3*/ robot.connect_joint("lfsh", "lflu", jpu, 'x', -90,  +90, -15 + rnd(X), JointType::symmetric, "L_shoulder_pitch", "R_shoulder_pitch", m.torque, params);
    /*4*/ robot.connect_joint("rflu", "rfll", jpl, 'x',   0, +180, +30 + rnd(X), JointType::normal   , "R_elbow_pitch"   , ""                , m.torque, params);
    /*5*/ robot.connect_joint("lflu", "lfll", jpl, 'x',   0, +180, +30 + rnd(X), JointType::symmetric, "L_elbow_pitch"   , "R_elbow_pitch"   , m.torque, params);

    /*rear legs*/
    /*6*/ robot.connect_joint("body", "rhsh", 0.0, 'Y', -90,  +90, + 1 + rnd(X), JointType::normal   , "R_hip_roll"      , ""                , m.torque, params);
    /*7*/ robot.connect_joint("body", "lhsh", 0.0, 'y', -90,  +90, + 1 + rnd(X), JointType::symmetric, "L_hip_roll"      , "R_hip_roll"      , m.torque, params);
    /*8*/ robot.connect_joint("rhsh", "rhlu", jpu, 'x', -90,  +90, -20 + rnd(X), JointType::normal   , "R_hip_pitch"     , ""                , m.torque, params);
    /*9*/ robot.connect_joint("lhsh", "lhlu", jpu, 'x', -90,  +90, -20 + rnd(X), JointType::symmetric, "L_hip_pitch"     , "R_hip_pitch"     , m.torque, params);
    /*A*/ robot.connect_joint("rhlu", "rhll", jpl, 'x',   0, +180, +20 + rnd(X), JointType::normal   , "R_knee_pitch"    , ""                , m.torque, params);
    /*B*/ robot.connect_joint("lhlu", "lhll", jpl, 'x',   0, +180, +20 + rnd(X), JointType::symmetric, "L_knee_pitch"    , "R_knee_pitch"    , m.torque, params);

    /* attach sensors */
    robot.attach_accel_sensor("body", /* keep original color = */true);

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(1.5, -0.5, 0.7), 140, -10, 0);
}





}} // namespace Robots::Hannah

#endif // HANNAH_DETAIL_RND_H_INCLUDED
