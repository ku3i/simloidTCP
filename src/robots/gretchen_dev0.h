#ifndef GRETCHEN_RANDOM_H_INCLUDED
#define GRETCHEN_RANDOM_H_INCLUDED

#include <basic/common.h>
#include <basic/color.h>
#include <build/robot.h>
#include <build/params.h>
#include <build/physics.h>

/** consider to put joint connections with desired joint ID into queue and create them all at once,
so that we assign specific ids during creation and are able to have methods for creating legs*/


namespace Robots {
namespace Gretchen {

    using common::rnd;

    const double range = 0.2;

    const Vector3 zero = {.0, .0, .0,};

    const struct MotorDimensions { //TODO randomize?
        double width  = 0.030;
        double height = 0.065;
        double depth  = 0.075;
        double offset = 0.010;
    } mot;

    const Vector3 motor_face_front_vertical   = {mot.width , mot.depth , mot.height};
    const Vector3 motor_face_front_horizontal = {mot.height, mot.depth , mot.width };
    const Vector3 motor_face_side_vertical    = {mot.depth , mot.width , mot.height};
    const Vector3 motor_face_side_horizontal  = {mot.depth , mot.height, mot.width };

    struct Capsule {
        Capsule(double l, double r) : len(l), rad(r) {}

        double len, rad; };

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

    struct GrtDev0Morphology
    {
        const Vector3 body;
        const Vector3 body_ext;
        const Vector3 leg_upper;
        const Vector3 knee;
        const Vector3 leg_lower;
        const Vector3 ankle;
        const Vector3 foot;

        const double dbl_bearing;

        const double hip_a;

        const double body_hip_joint_dist_xz;
        const double hip_legs_joint_dist_yz;

        const double zheight_start;

        /* TODO: consider increasing the range here */
        const double ground_contact_friction;

        /* weights */
        const struct Weights {
            double body;
            double hip;
            double leg_upper;
            double knee;
            double leg_lower;
            double ankle;
            double foot;
            double motor;
            double dbl_bearing;
        } weight_kg;

        const double torque;

        Color4 color_body;
        Color4 color_dark;
        Color4 color_light;

        GrtDev0Morphology(double amp)
        : body( rnd(.110, range, amp)
              , rnd(.010, range, amp)
              , rnd(.250, range, amp)
              )
        , body_ext( rnd(.045, range, amp)
                  , rnd(.010, range, amp)
                  , rnd(.170, range, amp)
                  )
        , leg_upper( rnd(.010, range, amp)
                   , rnd(.100, range, amp)
                   , rnd(.290, range, amp) // length
                   )
        , knee ( rnd(.040, range, amp)
               , rnd(.050, range, amp)
               , rnd(.100, range, amp) // length
               )
        , leg_lower( rnd(.010, range, amp)
                   , rnd(.060, range, amp)
                   , rnd(.225, range, amp) // length
                   )
        , ankle    ( rnd(.050, range, amp)  // width
                   , rnd(.080, range, amp)  // length
                   , rnd(.015, range, amp)  // height
                   )
        , foot     ( rnd(.085, range, amp) // width
                   , rnd(.220, range, amp) // length
                   , rnd(.005, range, amp) // height
                   )
        , dbl_bearing( rnd(.030, range, amp))
        , hip_a( rnd(.04, range, amp) ) //defined
        , body_hip_joint_dist_xz( rnd(.025, range, amp) )
        , hip_legs_joint_dist_yz ( rnd(.03, range, amp) )
        , zheight_start( body.z + leg_upper.z + leg_lower.z + foot.z - 0.13)
        , ground_contact_friction( rnd(constants::friction::sticky, range, amp) )
        , weight_kg({rnd(0.110, range, amp)  // body
                   , rnd(0.020, range, amp)  // hip without axis
                   , rnd(0.070, range, amp)  // leg_upper / thigh
                   , rnd(0.070, range, amp)  // knee without bearings
                   , rnd(0.035, range, amp)  // leg_lower / shank
                   , rnd(0.050, range, amp)  /**TODO*/ // ankle;
                   , rnd(0.050, range, amp)  /**TODO*/ // foot;
                   , rnd(0.195, range, amp)  // motor, incl, screws+nuts
                   , rnd(0.100, range, amp)  // dbl_bearing with axis
                   })
        , torque( rnd(5.0, range, amp) ) /**TODO*/
        , color_body ( rnd(.5, 1.0, amp), rnd(.5, 1.0, amp), rnd(.5, 1.0, amp), 1.0 )
        , color_dark ( rnd(.3, .50, amp), rnd(.3, .50, amp), rnd(.3, .50, amp), 1.0 )
        , color_light( rnd(.9, .10, amp), rnd(.9, .10, amp), rnd(.9, .10, amp), 1.0 )
        {
            dsPrint("Model Parameters:\n"
                    "\tBody       : m=%1.4f l=(%1.4f %1.4f %1.4f)\n"
                    "\tHip        : m=%1.4f l=(%1.4f)\n"
                    "\tLegs Upper : m=%1.4f l=(%1.4f %1.4f %1.4f)\n"
                    "\tLegs Lower : m=%1.4f l=(%1.4f %1.4f %1.4f)\n"
                    , weight_kg.body     , body.x       , body.y       , body.z
                    , weight_kg.hip , hip_a
                    , weight_kg.leg_upper, leg_upper.x  , leg_upper.y  , leg_upper.z
                    , weight_kg.leg_lower, leg_lower.x  , leg_lower.y  , leg_lower.z
                    );
        }

    };


void
create_grt_dev0_rnd(Robot& robot, std::vector<double> model_parameter)
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

    GrtDev0Morphology m(rnd_amp);
    ActuatorParameters params = Sensorimotor;
    params.randomize(range, rnd_amp);

    srand(time(NULL)); // reset usual seed for random number generator.

    dsPrint("Creating randomized grt_dev0 <3\n");

    const double X = 3*rnd_amp;

    /* body */
    Vector3 pos (.0, .0, m.zheight_start);
    robot.create_box("loto", pos, m.body, m.weight_kg.body, 0, m.color_body, true, constants::friction::hi); // lower torso

    /* attaching body extras */
    pos.x =  .5*(m.body.x + m.body_ext.x);
    pos.z += .5*(m.body.z - m.body_ext.z);
    robot.attach_box("loto", pos     , m.body_ext, 0.050/**TODO*/, 0, m.color_light, false );
    robot.attach_box("loto", pos._x(), m.body_ext, 0.050/**TODO*/, 0, m.color_light, false );

    /* attaching body motors */
    pos.y -= mot.offset;
    pos.z = m.zheight_start;
    pos.x = .5*m.body.x - m.body_hip_joint_dist_xz;
    robot.attach_box("loto", pos     , motor_face_front_vertical, m.weight_kg.motor, 0, colors::black_t, false );
    robot.attach_box("loto", pos._x(), motor_face_front_vertical, m.weight_kg.motor, 0, colors::black_t, false );

    const double hip_z = m.zheight_start - 0.5*m.body.z + m.body_hip_joint_dist_xz;
    pos.z = hip_z;
    pos.y += mot.offset;

    /* attaching bearing/axis */
    robot.attach_box("loto", pos     , m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("loto", pos._x(), m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );

    /* hips */
    pos.y = .5*m.body.y + m.hip_a;

    robot.create_box("lehi", pos     , m.hip_a, m.weight_kg.hip, 0, colors::orange_t, false ); // left hip
    robot.create_box("rihi", pos._x(), m.hip_a, m.weight_kg.hip, 0, colors::orange_t, false ); // right hip

    /* legs upper */
    pos.z = hip_z + m.hip_legs_joint_dist_yz - .5*m.leg_upper.z;
    pos.y += - 0.5*m.leg_upper.y + m.hip_legs_joint_dist_yz;
    pos.x += 1.5*m.hip_a + 0.5*m.leg_upper.x;

    const Vector3 motor_pos_hip  = {-mot.offset, .00,  .025/**TODO: check these values*/}; /**TODO randomize?*/
    const Vector3 motor_pos_knee = {-mot.offset,-.02, -.045};

    const Vector3 bearing_pos_hi = {0.0,+.5*m.leg_upper.y - .5*m.leg_lower.y
                                       ,+.5*m.leg_upper.z - .5*m.leg_lower.y};

    const Vector3 bearing_pos_kn = {0.0,-.5*m.leg_upper.y + .5*m.leg_lower.y
                                       ,-.5*m.leg_upper.z + .5*m.leg_lower.y};

    const Vector3 bearing_pos_lo = {0.0,+.25*m.leg_lower.y
                                       ,-.5*m.leg_lower.z + 0.025};//.25*m.leg_lower.y};

    robot.create_box("lelu", pos      , m.leg_upper, m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi); // left leg upper
    robot.create_box("rilu", pos._x() , m.leg_upper, m.weight_kg.leg_upper, 0, m.color_light, true, constants::friction::hi); // right leg upper


    /* attaching bearing/axis */
    robot.attach_box("lelu", pos      + bearing_pos_hi     , m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("rilu", pos._x() + bearing_pos_hi._x(), m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );

    robot.attach_box("lelu", pos      + bearing_pos_kn     , m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("rilu", pos._x() + bearing_pos_kn._x(), m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );


    /* attaching motors to upper legs */
    robot.attach_box("lelu", pos      + motor_pos_hip      , motor_face_side_horizontal, m.weight_kg.motor, 0, colors::black_t, false );
    robot.attach_box("lelu", pos      + motor_pos_knee     , motor_face_side_vertical  , m.weight_kg.motor, 0, colors::black_t, false );
    robot.attach_box("rilu", pos._x() + motor_pos_hip ._x(), motor_face_side_horizontal, m.weight_kg.motor, 0, colors::black_t, false );
    robot.attach_box("rilu", pos._x() + motor_pos_knee._x(), motor_face_side_vertical  , m.weight_kg.motor, 0, colors::black_t, false );


    /* legs lower */
    pos.z += -.5*m.leg_upper.z - .5*m.leg_lower.z;
    pos.y += -.5*m.leg_upper.y + .5*m.leg_lower.y;

    robot.create_box("lell", pos      , m.leg_lower, m.weight_kg.leg_lower, 0, m.color_body, true, constants::friction::hi); // left leg lower
    robot.create_box("rill", pos._x() , m.leg_lower, m.weight_kg.leg_lower, 0, m.color_body, true, constants::friction::hi); // right leg lower

    /* attach the knee parts */
    const Vector3 knee_pos = {.0, -.5*m.leg_lower.y + .5*m.knee.y + 0.0001/*just for visuals */, .5*m.leg_lower.z};
    robot.attach_box("lell", pos      + knee_pos, m.knee, m.weight_kg.knee, 0, colors::orange_t, false ); // left knee
    robot.attach_box("rill", pos._x() + knee_pos, m.knee, m.weight_kg.knee, 0, colors::orange_t, false ); // right knee

    /* attaching bearing/axis */
    robot.attach_box("lell", pos      + bearing_pos_lo     , m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("rill", pos._x() + bearing_pos_lo._x(), m.dbl_bearing, m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );

    /* attach the motors */
    robot.attach_box("lell", pos      + Vector3{-mot.offset,.0, .02}, motor_face_side_vertical, m.weight_kg.motor, 0, colors::black_t, false ); //motor
    robot.attach_box("rill", pos._x() + Vector3{+mot.offset,.0, .02}, motor_face_side_vertical, m.weight_kg.motor, 0, colors::black_t, false ); //motor

    /* ankles */
    pos.z += -.5*m.leg_lower.z + 0.025;//.5*m.ankle.z;
    pos.y -= -.25*m.leg_lower.y;

    robot.create_box("lean", pos      , m.ankle, m.weight_kg.ankle, 0, colors::orange_t, false );
    robot.create_box("rian", pos._x() , m.ankle, m.weight_kg.ankle, 0, colors::orange_t, false );

    /* feet */
    const double foot_bearing_height = .055 - .5*m.foot.z;
    const double foot_bearing_offset = 0.25*m.leg_lower.y;
    pos.z -= foot_bearing_height;
    pos.y -= foot_bearing_offset;
    robot.create_box("lefo", pos      , m.foot, m.weight_kg.foot, 0, m.color_body, true, m.ground_contact_friction);
    robot.create_box("rifo", pos._x() , m.foot, m.weight_kg.foot, 0, m.color_body, true, m.ground_contact_friction);

    const Vector3 bearing_pos_foot_front = pos + Vector3{.0, +.05 + foot_bearing_offset, foot_bearing_height};
    const Vector3 bearing_pos_foot_back  = pos + Vector3{.0, -.05 + foot_bearing_offset, foot_bearing_height};
    const Vector3 bearing_size = {m.dbl_bearing, .5*m.dbl_bearing, m.dbl_bearing};

    robot.attach_box("lefo", bearing_pos_foot_front     , bearing_size, .5*m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("rifo", bearing_pos_foot_front._x(), bearing_size, .5*m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("lefo", bearing_pos_foot_back      , bearing_size, .5*m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );
    robot.attach_box("rifo", bearing_pos_foot_back ._x(), bearing_size, .5*m.weight_kg.dbl_bearing, 0, colors::cyan_t, false );


    /* attach motors */
    pos += Vector3{.0,-.06, .5*mot.width+m.foot.z};
    robot.attach_box("lefo", pos      , motor_face_front_horizontal, m.weight_kg.motor, 0, colors::black_t, false );
    robot.attach_box("rifo", pos._x() , motor_face_front_horizontal, m.weight_kg.motor, 0, colors::black_t, false );

    /* relative joint positions */
    const Vector3 jpu = {.0, 0.5*m.leg_upper.y - m.hip_legs_joint_dist_yz, +.5*m.leg_upper.z - m.hip_legs_joint_dist_yz};
    const Vector3 jpl = {.0, .0, .5*m.leg_lower.z + .5*m.leg_lower.y };
    const Vector3 jpf = {.0, foot_bearing_offset, foot_bearing_height};

    /* connect by joints */
    robot.connect_joint("loto", "lehi", zero, 'y', - 90,  90,  -1 + rnd(X), JointType::normal,    "L_hip_roll"   , ""             , m.torque, params);
    robot.connect_joint("loto", "rihi", zero, 'Y', - 90,  90,  -1 + rnd(X), JointType::symmetric, "R_hip_roll"   , "L_hip_roll"   , m.torque, params);
    robot.connect_joint("lehi", "lelu", jpu , 'x', - 90,  90,  10 + rnd(X), JointType::normal,    "L_hip_pitch"  , ""             , m.torque, params);
    robot.connect_joint("rihi", "rilu", jpu , 'x', - 90,  90,  10 + rnd(X), JointType::symmetric, "R_hip_pitch"  , "L_hip_pitch"  , m.torque, params);
    robot.connect_joint("lelu", "lell", jpl , 'x', -135,   0, -20 + rnd(X), JointType::normal,    "L_knee_pitch" , ""             , m.torque, params);
    robot.connect_joint("rilu", "rill", jpl , 'x', -135,   0, -20 + rnd(X), JointType::symmetric, "R_knee_pitch" , "L_knee_pitch" , m.torque, params);
    robot.connect_joint("lell", "lean", zero, 'x', - 45,  45,  10 + rnd(X), JointType::normal,    "L_ankle_pitch", ""             , m.torque, params);
    robot.connect_joint("rill", "rian", zero, 'x', - 45,  45,  10 + rnd(X), JointType::symmetric, "R_ankle_pitch", "L_ankle_pitch", m.torque, params);
    robot.connect_joint("lean", "lefo", jpf , 'y', - 90,  90,   0 + rnd(X), JointType::normal,    "L_ankle_roll" , ""             , m.torque, params);
    robot.connect_joint("rian", "rifo", jpf , 'Y', - 90,  90,   0 + rnd(X), JointType::symmetric, "R_ankle_roll" , "L_ankle_roll" , m.torque, params);

    /* attach sensors */
    robot.attach_accel_sensor("loto", /* keep original color = */false);

    /* camera */
    robot.set_camera_center_on("loto");
    robot.setup_camera(Vector3(1.5, -0.5, 0.7), 140, -10, 0);
}

}} /* namespace Robots::Gretchen */

#endif /* GRETCHEN_RANDOM_H_INCLUDED */
