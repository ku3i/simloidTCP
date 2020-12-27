#include <build/bioloid.h>
#include <scenes/scenes.h>
#include <robots/crawler.h>
#include <robots/biped.h>
#include <robots/karl_sims.h>
#include <robots/fourlegged.h>
#include <robots/standard.h>
#include <robots/nolegs.h>
#include <robots/hannah.h>
#include <robots/hannah_random.h>
#include <robots/hannah_detail_rnd.h>
#include <robots/gretchen.h>
#include <robots/gretchen_dev0.h>

void
Bioloid::create_scene(Obstacle& obstacles, Landscape& landscape)
{
    assert(obstacles.number_of_objects() == 0);
    dsPrint("Creating scene: ");
    switch (global_conf.scene)
    {
        case 0: Scenes::create_empty_world();           break;
        case 1: Scenes::create_hurdles(obstacles);      break;
        case 2: Scenes::create_hills(landscape);        break;
        case 3: Scenes::create_shaky_ground(obstacles); break;
        case 4: Scenes::create_stairways(obstacles);    break;
        case 5: Scenes::create_plates(obstacles);       break;
        case 6: Scenes::create_random(obstacles);       break;
        default: dsPrint("Warning: Wrong scene index number.\n");
    }
}

/* when no model number is given explicitly, use the one from the global configuration. */
void Bioloid::create_robot(Robot& robot) { create_robot(robot, global_conf.robot, std::vector<double>{}); }

void Bioloid::create_robot(Robot& robot, int index_number, std::vector<double> params)
{
    dsPrint("Creating robot with index number %d: \n", index_number);
    switch (index_number)
    {
        case 10: Robots::KarlSims  ::create_tadpole_0       (robot); break;
        case 11: Robots::KarlSims  ::create_tadpole_1       (robot); break;

        case 20: Robots::Crawler   ::create_crawler_0       (robot); break;

        case 30: Robots::Fourlegged::create_wildcat_0       (robot); break;
        case 31: Robots::Fourlegged::create_wildcat_1       (robot); break;
        case 32: Robots::Hannah    ::create_hannah_0        (robot); break;
        case 33: Robots::Hannah    ::create_hannah_1        (robot); break;
        case 34: Robots::Hannah    ::create_hannah_2        (robot); break;

        case 37: Robots::HannahRand::create_random_hannah   (robot, params); break;
        case 38: Robots::HannahRandDetail::create_hannah_detail_random(robot, params); break;

        case 40: Robots::Biped     ::create_ostrich         (robot); break;

        case 50: Robots::Biped     ::create_humanoid0       (robot); break;
        case 51: Robots::Biped     ::create_humanoid1       (robot); break;

        case 55: Robots::Gretchen  ::create_gretchen0       (robot, params); break;
        case 56: Robots::Gretchen  ::create_grt_dev0_rnd    (robot, params); break;

        case 60: Robots::Nolegs    ::create_worm            (robot); break;

        case 80: Robots::Hannah    ::create_hannah_leg      (robot); break;
        case 81: Robots::HannahRand::create_motor_param_test(robot); break;
        case 82: Robots::HannahRand::create_motor_param_test_vert(robot); break;

        case 90: Robots::Standard  ::create_pendulum        (robot); break;
        case 91: Robots::Standard  ::create_double_pendulum (robot); break;
        case 92: Robots::Standard  ::create_rotor_horizontal(robot); break;
        case 93: Robots::Standard  ::create_rotor_vertical  (robot); break;
        case 94: Robots::Standard  ::create_rotor_axial     (robot); break;

        case 95: Robots::Standard  ::create_pendulum (robot, false); break;
        case 96: Robots::Standard  ::create_pendulum_gmes_ed(robot); break;

        default:
            dsError("Wrong robot index number (%d)!\n", index_number);
            break;
    }

    robot.print_statistics();
}
