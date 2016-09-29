#ifndef CRAWLER_H_INCLUDED
#define CRAWLER_H_INCLUDED

#include <basic/constants.h>
#include <build/robot.h>
#include <build/physics.h>

namespace Robots {
namespace Crawler {

/* body */
const double length_torso  = .15;

/* arm */
const double breadth_arm   = .1*length_torso;
const double length_arm    = length_torso*1.1;
const double width_arm     = 0.62*length_torso;

const double zheight_start = .05;

void
create_crawler_0(Robot& robot)
{
    dsPrint("CRAWLER\n");

    Vector3 pos;
    Vector3 len(length_torso);

    /* body */
    pos.x = .0;
    pos.y = 0.5*length_torso + length_arm + 0.5*breadth_arm;
    pos.z = 0.5*length_torso + zheight_start;

    robot.create_box("body", pos, len, .0, constants::materials::normal, colors::white, true, constants::friction::lo);

    /* arm */
    pos.y = 0.5*length_arm - 0.5*breadth_arm;
    pos.z = length_torso + zheight_start + 0.5*breadth_arm;

    len.x = width_arm;
    len.y = length_arm;
    len.z = breadth_arm;

    robot.create_box("arm_upper", pos, len, .0, constants::materials::body, colors::white, true, constants::friction::lo);

    pos.y = -breadth_arm;
    pos.z = length_torso + zheight_start - 0.5*length_arm;

    len.x = width_arm;
    len.y = breadth_arm;
    len.z = length_arm;

    robot.create_box("arm_lower", pos, len, .0, constants::materials::body, colors::black, true, constants::friction::hi);

    /* connect joints */
    robot.connect_joint("body"     , "arm_upper", .0, .5*length_arm + 0.5*breadth_arm, .0, 'x', -60, +150, 0, JointType::normal, "shoulderpitch", "", 3);
    robot.connect_joint("arm_upper", "arm_lower", .0, .0, .5*length_arm + 0.5*breadth_arm, 'x', -60, +150, 0, JointType::normal, "elbowpitch"   , "", 3);

    /* attach sensors */
    robot.attach_accel_sensor("body");
}

}} // namespace Robots::Crawler


#endif // CRAWLER_H_INCLUDED
