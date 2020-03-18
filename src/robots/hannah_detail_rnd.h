#ifndef HANNAH_DETAIL_RND_H_INCLUDED
#define HANNAH_DETAIL_RND_H_INCLUDED

#include <sstream>

#include <basic/common.h>
#include <basic/color.h>
#include <basic/capsule.h>
#include <build/robot.h>
#include <build/params.h>
#include <build/physics.h>


namespace Robots {
namespace HannahRandDetail {

    using common::rnd;
    using common::rndg;

    const double range = 0.2;

    const struct MotorDimensions { //TODO randomize!
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



    ActuatorParameters Sensorimotor {{/* bristle_displ_max = */ +8.71005098e-03, //+1.00000000e-03,
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

    struct HannahMorphology
    {
        const Vector3 body;
        const Vector3 leg_upper;
        const Capsule leg_lower;
        const Vector3 knee;
        const Capsule foot;

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
            double foot;
            double motor;
            double dbl_bearing;
        } weight_kg;

        const double torque;

        Color4 color_body;
        Color4 color_dark;
        Color4 color_light;

        HannahMorphology(double /*random*/r , double /*growth*/g)
        : body( rndg(.250, range, r, g)
              , rndg(.500, range, r, g)
              , rndg(.120, range, r, g)
              )
        , leg_upper( rndg(.010, range, r, g)
                   , rndg(.060, range, r, g)
                   , rndg(.320, range, r, g) // length
                   )
        , leg_lower( 3
                   , rndg(0.30, range, r, g) // length
                   , rndg(0.01, range, r, g) // radius
                   )
        , knee( rndg(0.02, range, r, g)
              , rndg(0.04, range, r, g)
              , rndg(0.04, range, r, g)
              )
        , foot( 3
              , 0. //rndg(0.015, range, r, g) // length
              , rndg(0.015, range, r, g)
              )
        , dbl_bearing( rndg(.030, range, r, g))
        , shoulder_a( rndg(.04, range, r, g) )
        , body_shld_joint_dist_xz( rndg(.028, range, r, g) )
        , shld_legs_joint_dist_z ( rndg(.016, range, r, g) )
        , knee_y_offset( rndg(.034, range, r, g) )
        , zheight_start( leg_upper.z + leg_lower.len + leg_lower.rad - 0.25*body.z)
        , ground_contact_friction( rndg(constants::friction::sticky, 2*range, r, g) )
        , weight_kg({rndg(1.000, range, r, g) // body TODO specify
                   , rndg(0.030, range, r, g) // shoulder TODO specify
                   , rndg(0.055, range, r, g) // leg_upper TODO specify
                   , rndg(0.100, range, r, g) // leg_lower TODO specify, make the real leg weight more and the leg shorter, in order to back-drive */
                   , rndg(0.050, range, r, g) // knee TODO
                   , rndg(0.010, range, r, g) // TODO specify
                   , rndg(0.195, range, r, g) // motor, incl. screws+nuts
                   , rndg(0.100, range, r, g) // dbl_bearing with axis
                   })
        , torque( rndg(5.0, range, r, g) )
        , color_body ( rnd(.5, 1.0, r), rnd(.5, 1.0, r), rnd(.5, 1.0, r), 1.0 )
        , color_dark ( rnd(.3, .50, r), rnd(.3, .50, r), rnd(.3, .50, r), 1.0 )
        , color_light( rnd(.9, .10, r), rnd(.9, .10, r), rnd(.9, .10, r), 1.0 )
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
    robot.attach_box(name+"lu", Vector3{-sign*mot.offset,.0,+.04/**TODO*/}, motor_face_side, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box(name+"lu", Vector3{-sign*mot.offset,.0,-.04/**TODO*/}, motor_face_side, m.weight_kg.motor, 0, colors::black_t, false);

    const Vector3 upper_bearing_pos = Vector3{.0, .0, +.5*m.leg_upper.z - m.shld_legs_joint_dist_z};
    const Vector3 lower_bearing_pos = Vector3{.0, -.25*m.leg_upper.y + .5*m.leg_lower.rad, -.5*m.leg_upper.z + m.knee_y_offset -m.leg_lower.rad };

    /* attach bearings */
    robot.attach_box(name+"lu", upper_bearing_pos, m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);
    robot.attach_box(name+"lu", lower_bearing_pos, m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false);

    const Vector3 knee_pos = Vector3{ .0, -.01, .5*m.leg_lower.len + 0.02};
    /* attach knee */
    robot.attach_box(name+"ll", knee_pos, m.knee, m.weight_kg.knee, 0, colors::magenta_t, false);

    /* attach foot */
    robot.attach_segment(name+"ll", -Vector3{ .0, .0, .5*m.leg_lower.len}, m.foot, m.weight_kg.foot, 0, colors::yellow_t, true, m.ground_contact_friction);
}

void
create_hannah_detail_random(Robot& robot, std::vector<double> model_parameter)
{
    unsigned rnd_instance = 0;
    double   rnd_amp = .0;
    double   growth = 1.0;

    if (model_parameter.size() == 3) {
        rnd_instance = static_cast<unsigned>(model_parameter[0]);
        rnd_amp      = model_parameter[1];
        growth       = model_parameter[2];
        dsPrint("Using model instance %u and random amp %lf and growth %lf", rnd_instance, rnd_amp, growth);
    }
    else if (model_parameter.size() == 0)
        dsPrint("No model parameters provided, taking defaults.");
    else
        dsError("Wrong number of model parameters.");

    if (rnd_instance != 0)
        srand(rnd_instance);

    HannahMorphology m(rnd_amp, growth);
    ActuatorParameters params = Sensorimotor;
    params.randomize(range, rnd_amp);

    srand(time(NULL)); // reset usual seed for random number generator.

    dsPrint("Creating randomized Hannah <3\n");

    const double X = 3+3*rnd_amp;

    std::ostringstream descr;
    descr << "Hannah Randomized\n";
    descr << "weight_body="       << m.weight_kg.body              << "\n";
    descr << "weight_leg_upper="  << m.weight_kg.leg_upper         << "\n";
    descr << "weight_leg_lower="  << m.weight_kg.leg_lower         << "\n";
    descr << "length_body="       << m.body.y                      << "\n";
    descr << "length_leg_upper="  << m.leg_upper.z                 << "\n";
    descr << "length_leg_lower="  << m.leg_lower.len               << "\n";
    descr << "leg_ratio="         << m.leg_lower.len/m.leg_upper.z << "\n";
    descr << "knee_y_offset="     << m.knee_y_offset               << "\n";

    descr << "torque="            << m.torque                      << "\n";
    descr << "ground_friction="   << m.ground_contact_friction     << "\n";
    descr << "bristle_displ_max=" << params.bristle_displ_max      << "\n";
    descr << "bristle_stiffness=" << params.bristle_stiffness      << "\n";
    descr << "sticking_friction=" << params.sticking_friction      << "\n";
    descr << "coulomb_friction="  << params.coulomb_friction       << "\n";
    descr << "fluid_friction="    << params.fluid_friction         << "\n";
    descr << "stiction_range="    << params.stiction_range         << "\n";
    descr << "V_in="              << params.V_in                   << "\n";
    descr << "kB="                << params.kB                     << "\n";
    descr << "kM="                << params.kM                     << "\n";
    descr << "R_i_inv="           << params.R_i_inv                << "\n";


    robot.description = descr.str();

    /* body */
    Vector3 body_pos (.0, .0, m.zheight_start);
    robot.create_box("body", body_pos, m.body, m.weight_kg.body, 0, m.color_body, true, constants::friction::hi); // body

    /* attach motors */
    const Vector3 pos_motor = /**TODO*/Vector3{-.125*m.body.x, -.5*m.body.y + mot.offset, .0};
    robot.attach_box("body", pos_motor          , motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box("body", pos_motor._x()     , motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box("body", pos_motor     ._y(), motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);
    robot.attach_box("body", pos_motor._x()._y(), motor_face_front, m.weight_kg.motor, 0, colors::black_t, false);

    /* attach bearings */
    const Vector3 bearing_pos = /**TODO*/Vector3{ .5*m.body.x - m.body_shld_joint_dist_xz
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
    /*2*/ robot.connect_joint("rfsh", "rflu", jpu, 'x', -90,  +90, -25 + rnd(X), JointType::normal   , "R_shoulder_pitch", ""                , m.torque, params);
    /*3*/ robot.connect_joint("lfsh", "lflu", jpu, 'x', -90,  +90, -25 + rnd(X), JointType::symmetric, "L_shoulder_pitch", "R_shoulder_pitch", m.torque, params);
    /*4*/ robot.connect_joint("rflu", "rfll", jpl, 'x',   0, +180, +40 + rnd(X), JointType::normal   , "R_elbow_pitch"   , ""                , m.torque, params);
    /*5*/ robot.connect_joint("lflu", "lfll", jpl, 'x',   0, +180, +40 + rnd(X), JointType::symmetric, "L_elbow_pitch"   , "R_elbow_pitch"   , m.torque, params);

    /*rear legs*/
    /*6*/ robot.connect_joint("body", "rhsh", 0.0, 'Y', -90,  +90, + 1 + rnd(X), JointType::normal   , "R_hip_roll"      , ""                , m.torque, params);
    /*7*/ robot.connect_joint("body", "lhsh", 0.0, 'y', -90,  +90, + 1 + rnd(X), JointType::symmetric, "L_hip_roll"      , "R_hip_roll"      , m.torque, params);
    /*8*/ robot.connect_joint("rhsh", "rhlu", jpu, 'x', -90,  +90, -40 + rnd(X), JointType::normal   , "R_hip_pitch"     , ""                , m.torque, params);
    /*9*/ robot.connect_joint("lhsh", "lhlu", jpu, 'x', -90,  +90, -40 + rnd(X), JointType::symmetric, "L_hip_pitch"     , "R_hip_pitch"     , m.torque, params);
    /*A*/ robot.connect_joint("rhlu", "rhll", jpl, 'x',   0, +180, +40 + rnd(X), JointType::normal   , "R_knee_pitch"    , ""                , m.torque, params);
    /*B*/ robot.connect_joint("lhlu", "lhll", jpl, 'x',   0, +180, +40 + rnd(X), JointType::symmetric, "L_knee_pitch"    , "R_knee_pitch"    , m.torque, params);

    /* attach sensors */
    robot.attach_accel_sensor("body", /* keep original color = */true);

    /* camera */
    robot.set_camera_center_on("body");
    robot.setup_camera(Vector3(1.5, -0.5, 0.7), 155, -5, 0);
}





}} // namespace Robots::Hannah

#endif // HANNAH_DETAIL_RND_H_INCLUDED
