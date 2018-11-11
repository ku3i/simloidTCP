#ifndef GRETCHEN_H_INCLUDED
#define GRETCHEN_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

/**CHECKLIST:
    + all frictions  set correctly?
    + all collisions set correctly?
    + all materials/weights set correctly?
    randomize start position
*/

namespace Robots {
namespace Gretchen {

    struct CapConf {
        CapConf(Capsule d, Vector3 p) : dim(d), pos(p) {}

        Capsule dim;
        Vector3 pos;
    };

    struct BoxConf {
        BoxConf(Vector3 d, Vector3 p) : dim(d), pos(p) {}

        Vector3 dim;
        Vector3 pos;
    };

void
create_gretchen0(Robot& robot, std::vector<double> /*model_parameter*/)
{
    dsPrint("Creating Gretchen...");
    const double friction = constants::friction::hi;

    /* shortening knees!! */
    const double unit_len = 0.23;

    const double D = 0.10;

    double X = 2*rnd(1.0);//5*rnd(1.0);

    dsPrint("with random factor: %1.3f", X);

    /* head */
    const CapConf head         ( {3, .0         , unit_len/2    }, {0.           , 0., unit_len*4.50 + D} );
    const CapConf neck         ( {3, .0         , unit_len/10   }, {0.           , 0., unit_len*4.00 + D} );
    //TODO NECK!!!

    /* torso */
    const CapConf torso_upper  ( {3, unit_len/2 , .9*unit_len/2 }, {0.           , -0.01, unit_len*3.25 + D} );
    const CapConf torso_middle ( {3, 0.         , unit_len/4    }, {0.           , -0.01, unit_len*2.75 + D} );
    const CapConf torso_lower  ( {3, 0.         , unit_len*5/12.}, {0.           , -0.01, unit_len*2.50 + D} );

    /**For some reasons giving the shoulder dir=1, the PID was not working. weird*/
    const CapConf shoulder     ( {2, unit_len/6 , unit_len/10   }, {unit_len*0.65, 0., unit_len*3.50 + D + unit_len/6} );
    const CapConf hip          ( {2, unit_len/6 , unit_len/10   }, {unit_len*0.30, 0., unit_len*2.00 + D} );

    /* arms */
    const CapConf arm_upper0   ( {3, unit_len/2-unit_len/6 , unit_len/6    }, {unit_len*0.65 , 0., unit_len*3.50 + D} );
    const CapConf arm_upper1   ( {3, unit_len/2-unit_len/6 , unit_len/6    }, {unit_len*0.65 , 0., unit_len*3.25 + D} );
    const CapConf arm_lower    ( {3, unit_len  -unit_len/7 , unit_len/7    }, {unit_len*0.65 , 0., unit_len*2.50 + D} );
    /* legs */
    double leg_offset = unit_len/6 - unit_len/5;
    const CapConf leg_upper0   ( {3, unit_len/2 , unit_len/5    }, {unit_len*0.25, 0., unit_len*1.75 + D} );
    const CapConf leg_upper1   ( {3, unit_len/2 , unit_len/5    }, {unit_len*0.25, 0., unit_len*1.25 + D} );
    const CapConf leg_lower    ( {3, unit_len   , unit_len/6    }, {unit_len*0.25, leg_offset, unit_len*0.50 + D} );
    /* ankles */
    const CapConf ankle        ( {3, .0         , unit_len/8    }, {unit_len*0.25, 0., 0 + D} );
    /* feet */
    const BoxConf foot         ( {unit_len/4, unit_len*0.7, .030}, {unit_len*0.25, -0.03, -unit_len*0.25 + D} );


    /* body */
    robot.create_segment("head", head        .pos     , head        .dim, 0, constants::materials::light , colors::black, true , friction);
    robot.create_segment("neck", neck        .pos     , neck        .dim, 0, constants::materials::light , colors::invisible, false, friction);
    robot.create_segment("upto", torso_upper .pos     , torso_upper .dim, 0, constants::materials::light , colors::white, true , friction);
    robot.create_segment("mito", torso_middle.pos     , torso_middle.dim, 0, constants::materials::light , colors::black, false, friction);
    robot.create_segment("loto", torso_lower .pos     , torso_lower .dim, 0, constants::materials::light , colors::white, false, friction);
    /* shoulders */
    robot.create_segment("lshd", shoulder    .pos     , shoulder    .dim, 0, constants::materials::normal, colors::black, false, friction);
    robot.create_segment("rshd", shoulder    .pos._x(), shoulder    .dim, 0, constants::materials::normal, colors::black, false, friction);
    /* hips */
    robot.create_segment("lhip", hip         .pos     , hip         .dim, 0, constants::materials::normal, colors::black, false, friction);
    robot.create_segment("rhip", hip         .pos._x(), hip         .dim, 0, constants::materials::normal, colors::black, false, friction);
    /* arms upper */
    robot.create_segment("lau0", arm_upper0  .pos     , arm_upper0  .dim, 0, constants::materials::light , colors::white, false, friction);
    robot.create_segment("rau0", arm_upper0  .pos._x(), arm_upper0  .dim, 0, constants::materials::light , colors::white, false, friction);
    robot.create_segment("lau1", arm_upper1  .pos     , arm_upper1  .dim, 0, constants::materials::light , colors::gray, false, friction);
    robot.create_segment("rau1", arm_upper1  .pos._x(), arm_upper1  .dim, 0, constants::materials::light , colors::gray, false, friction);
    /* arms lower */
    robot.create_segment("lalo", arm_lower   .pos     , arm_lower   .dim, 0, constants::materials::light , colors::black, true , friction);
    robot.create_segment("ralo", arm_lower   .pos._x(), arm_lower   .dim, 0, constants::materials::light , colors::black, true , friction);
    /* legs upper */
    robot.create_segment("llu0", leg_upper0  .pos     , leg_upper0  .dim, 0, constants::materials::normal, colors::white, true , friction);
    robot.create_segment("rlu0", leg_upper0  .pos._x(), leg_upper0  .dim, 0, constants::materials::normal, colors::white, true , friction);
    robot.create_segment("llu1", leg_upper1  .pos     , leg_upper1  .dim, 0, constants::materials::normal, colors::gray, true , friction);
    robot.create_segment("rlu1", leg_upper1  .pos._x(), leg_upper1  .dim, 0, constants::materials::normal, colors::gray, true , friction);
    /* legs lower */
    robot.create_segment("lllo", leg_lower   .pos     , leg_lower   .dim, 0, constants::materials::normal, colors::black, true , friction);
    robot.create_segment("rllo", leg_lower   .pos._x(), leg_lower   .dim, 0, constants::materials::normal, colors::black, true , friction);

    /* ankles */
    robot.create_segment("lean", ankle       .pos     , ankle       .dim, 0, constants::materials::normal, colors::black, false, friction);
    robot.create_segment("rian", ankle       .pos._x(), ankle       .dim, 0, constants::materials::normal, colors::black, false, friction);

    /* feet */
    robot.create_box    ("left", foot        .pos     , foot        .dim, 0, constants::materials::heavy , colors::white, true , friction);
    robot.create_box    ("rift", foot        .pos._x(), foot        .dim, 0, constants::materials::heavy , colors::white, true , friction);

    //robot.attach_box("upto", torso_upper .pos, {0.0001,1.6,1.6}, 0.1, 0.0, {1.0,  0.0,  1.0, 0.25}, false);

    //robot.attach_segment("left", pos, 3, 0, 0.02, 0, constants::materials::heavy, colors::orange, true, friction );
    Vector3 foot_joint_pos = Vector3{0.5*foot.dim.y - leg_lower.dim.rad, foot.dim.z+ankle.dim.rad, 0.0};

    /* connect with joints */
    robot.connect_joint("neck", "head", .0, .0, -.5*head.dim.len - 0.5*head.dim.rad, 'x',  -60,  +45, -10 + rnd(X), JointType::normal   , "neck_pitch"    , "");
    robot.connect_joint("upto", "neck", .0, .0, +.5*head.dim.rad                   , 'y',  -45,  +45,   0 + rnd(X), JointType::normal   , "neck_roll"     , "");
    robot.connect_joint("upto", "mito", .0, .0,  .0,                                 'y',  -45,  +45,   0 + rnd(X), JointType::normal   , "waistroll"     , "", 10);
    robot.connect_joint("mito", "loto", .0, .0, .25*unit_len,                        'x',  -10,  +60,  -5 + rnd(X), JointType::normal   , "waistpitch"    , "", 10);

    robot.connect_joint("upto", "lshd", .0, .0,  .0,                                 'x', -120,  +60,   0 + rnd(X), JointType::normal   , "lshoulderpitch", ""              ); // 180 deg is not enough
    robot.connect_joint("upto", "rshd", .0, .0,  .0,                                 'x', -120,  +60,   0 + rnd(X), JointType::symmetric, "rshoulderpitch", "lshoulderpitch");

    robot.connect_joint("lshd", "lau0", .0, .0, +.50*arm_upper0.dim.len,             'y',  -90, +150, +20 + rnd(X), JointType::normal   , "lshoulderroll" , ""              ); //TODO joint range
    robot.connect_joint("rshd", "rau0", .0, .0, +.50*arm_upper0.dim.len,             'Y',  -90, +150, +20 + rnd(X), JointType::symmetric, "rshoulderroll" , "lshoulderroll" );

    robot.connect_joint("lau0", "lau1", .0, .0, +.25*arm_upper1.dim.len,             'Z',  -90,  +90,  10 + rnd(X), JointType::normal   , "lshoulderyaw"  , ""              );
    robot.connect_joint("rau0", "rau1", .0, .0, +.25*arm_upper1.dim.len,             'z',  -90,  +90,  10 + rnd(X), JointType::symmetric, "rshoulderyaw"  , "lshoulderyaw"  );

    robot.connect_joint("lau1", "lalo", .0, .0, +.50*arm_lower.dim.len,              'x',    0,  120,  30 + rnd(X), JointType::normal   , "lelbowpitch"   , ""              );
    robot.connect_joint("rau1", "ralo", .0, .0, +.50*arm_lower.dim.len,              'x',    0,  120,  30 + rnd(X), JointType::symmetric, "relbowpitch"   , "lelbowpitch"   );


    robot.connect_joint("loto", "lhip", .0, .0, .0                   ,               'x',  -30,   90,   8 + rnd(X), JointType::normal   , "lhippitch"     , ""              , 10);
    robot.connect_joint("loto", "rhip", .0, .0, .0                   ,               'x',  -30,   90,   8 + rnd(X), JointType::symmetric, "rhippitch"     , "lhippitch"     , 10);

    robot.connect_joint("lhip", "llu0", .0, .0, +.50*leg_upper0.dim.len,             'y',  -90,   90,   1 + rnd(X), JointType::normal   , "lhiproll"      , ""              , 10);
    robot.connect_joint("rhip", "rlu0", .0, .0, +.50*leg_upper0.dim.len,             'Y',  -90,   90,   1 + rnd(X), JointType::symmetric, "rhiproll"      , "lhiproll"      , 10);

    robot.connect_joint("llu0", "llu1", .0, .0, +.50*leg_upper1.dim.len,             'Z',  -30,   60,   0 + rnd(X), JointType::normal   , "lhipyaw"       , ""              , 10);
    robot.connect_joint("rlu0", "rlu1", .0, .0, +.50*leg_upper1.dim.len,             'z',  -30,   60,   0 + rnd(X), JointType::symmetric, "rhipyaw"       , "lhipyaw"       , 10);

    robot.connect_joint("llu1", "lllo", .0, .0, +.50*leg_lower.dim.len,              'x', -120,    0, -16 + rnd(X), JointType::normal   , "lkneepitch"    , ""              , 10);
    robot.connect_joint("rlu1", "rllo", .0, .0, +.50*leg_lower.dim.len,              'x', -120,    0, -16 + rnd(X), JointType::symmetric, "rkneepitch"    , "lkneepitch"    , 10);

    robot.connect_joint("lllo", "lean", .0, .0, .0                                 , 'x',  -45,   45,  9.0 + rnd(X), JointType::normal   , "lanklepitch"   , ""              , 10);
    robot.connect_joint("rllo", "rian", .0, .0, .0                                 , 'x',  -45,   45,  9.0 + rnd(X), JointType::symmetric, "ranklepitch"   , "lanklepitch"   , 10);

    robot.connect_joint("lean", "left", .0, foot_joint_pos.x, foot_joint_pos.y,      'y',  -45,   45,   0 + rnd(X), JointType::normal   , "lankleroll"    , ""              , 10);
    robot.connect_joint("rian", "rift", .0, foot_joint_pos.x, foot_joint_pos.y,      'Y',  -45,   45,   0 + rnd(X), JointType::symmetric, "rankleroll"    , "lankleroll"    , 10);

    /* attach sensors */
    robot.attach_accel_sensor("head");

    /* camera */
    robot.set_camera_center_on("upto");
    robot.setup_camera(Vector3(1.0, -0.9, 0.9), 123, -25, 0);

}

}} // namespace Robots::Gretchen


#endif // GRETCHEN_H_INCLUDED
