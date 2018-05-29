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

void
Bioloid::create_scene(Obstacle& obstacles, Landscape& landscape)
{
    dsPrint("Creating scene: ");
    switch (global_conf.scene)
    {
        case 0: Scenes::create_empty_world();           break;
        case 1: Scenes::create_hurdles(obstacles);      break;
        case 2: Scenes::create_hills(landscape);        break;
        case 3: Scenes::create_shaky_ground(obstacles); break;
        case 4: Scenes::create_stairways(obstacles);    break;

        default: dsPrint("Warning: Wrong scene index number.\n");
    }
}

/* when no model number is given explicitly, use the one from the global configuration. */
void Bioloid::create_robot(Robot& robot) { create_robot(robot, global_conf.robot, 0, .0); }

void Bioloid::create_robot(Robot& robot, int index_number, unsigned rnd_instance, double rnd_amp)
{
    dsPrint("Creating robot with index number %d: \n", index_number);
    switch (index_number)
    {
        case 10: Robots::KarlSims  ::create_tadpole_0       (robot); break;
        case 11: Robots::KarlSims  ::create_tadpole_1       (robot); break;

        case 20: Robots::Crawler   ::create_crawler_0       (robot); break;

        case 30: Robots::Fourlegged::create_wildcat_0       (robot); break;
        case 31: Robots::Fourlegged::create_wildcat_1       (robot); break;
        case 32: Robots::Hannah    ::create_hannah          (robot); break;

        case 37: Robots::HannahRand::create_random_hannah   (robot, rnd_instance, rnd_amp); break;

        case 40: Robots::Biped     ::create_ostrich         (robot); break;

        case 50: Robots::Biped     ::create_humanoid0       (robot); break;
        case 51: Robots::Biped     ::create_humanoid1       (robot); break;

        case 60: Robots::Nolegs    ::create_worm            (robot); break;

        case 80: Robots::Hannah    ::create_hannah_leg      (robot); break;

        case 90: Robots::Standard  ::create_pendulum        (robot); break;
        case 91: Robots::Standard  ::create_double_pendulum (robot); break;
        case 92: Robots::Standard  ::create_rotor_horizontal(robot); break;
        case 93: Robots::Standard  ::create_rotor_vertical  (robot); break;
        case 94: Robots::Standard  ::create_rotor_axial     (robot); break;

        case 95: Robots::Standard  ::create_pendulum (robot, false); break;

        default:
            dsError("Wrong robot index number (%d)!\n", index_number);
            break;
    }

    robot.print_statistics();
}
