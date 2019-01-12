#include <scenes/scenes.h>

void
Scenes::create_empty_world()
{
    dsPrint("Flat world.\n");
}

void
Scenes::create_hurdles(Obstacle& obstacles)
{
    dsPrint("Hurdles.\n");
    Vector3 pos(.0);
    Vector3 len(1.5, 0.25, 0.0);
    double friction = 10.0;

    for (unsigned int i = 1; i < 10; ++i) // indented to start at 1
    {
        len.z = 0.002*i;
        pos.y = -1.0*i - 0.5; // remember: -i (unsigned!) => very large => kills ODE
        pos.z = 0.5*len.z + 0.001;
        obstacles.create_fixed_box("", pos, len, .0, constants::materials::rock, colors::white, friction);
    }
}

void
Scenes::create_hills(Landscape& landscape)
{
    dsPrint("Height field.\n");
    Color4 color(0.6, 1.0, 0.3, 0.3);
    Vector3 pos(0.0, -5.0, 0.0);
    //TODO size
    landscape.create_heightfield("Cosine Hills", pos, color);
}

void
Scenes::create_shaky_ground(Obstacle& obstacles)
{
    dsPrint("Shaky Ground.\n");
    Vector3 pos, len;
    Color4 color;
    double friction = 10.0;

    for (unsigned int i = 0; i < 200; ++i) // Don't change number of stones, TODO test if this still crashes.
    {
        len.x = common::getRandomDouble(0.02, 0.04);
        len.y = common::getRandomDouble(0.02, 0.04);
        len.z = common::getRandomDouble(0.02, 0.04);

        pos.x = common::getRandomDouble(-0.5, 0.5);
        pos.y = common::getRandomDouble( -10,-1.0);
        pos.z = common::getRandomDouble( 0.1, 0.5) + 0.5 * len.z;

        color = common::getRandomDouble(0.3, 0.7);

        obstacles.create_box("", pos, len, .0, constants::materials::rock, color, friction);

    }
}

void
Scenes::create_stairways(Obstacle& obstacles)
{
    dsPrint("Stairways.\n");
    Vector3 len(1.0, 0.30, 0.00);
    Vector3 pos(.0,-1.0, .0);
    double friction = 1000.0;
    const unsigned int max_stairs = 64;
    const double stepsize = 0.005;

    pos.z = 0.5*stepsize;
    for (unsigned int i = 0; i < max_stairs; ++i)
    {
        pos.y -= len.y - 0.01;
        len.z  = 0.002*i+stepsize;
        pos.z += len.z;
        len.x  = 1.0 + i*0.1;
        obstacles.create_fixed_box("", pos, len, .0, constants::materials::rock, colors::white, friction);
    }

}

void
Scenes::create_plates(Obstacle& obstacles)
{
    dsPrint("Ground with plates.\n");

    const int maxj = 5;
    const int maxi = 5;
    for (int i = 0; i < maxi; ++i)
        for (int j = 0; j < maxj; ++j)
        {
            Vector3 pos, len;
            Color4 color;
            double friction = 10.0;

            len.x = common::getRandomDouble(0.100, 0.100 + 0.025*(j+1));
            len.y = common::getRandomDouble(0.100, 0.100 + 0.025*(j+1));
            len.z = common::getRandomDouble(0.002, 0.002+0.003*(j+1));

            pos.x = (i - maxi/2) * 0.25 + common::getRandomDouble(-len.x/2, +len.x/2);
            pos.y =  -1.5 + (j * -0.25) + common::getRandomDouble(-len.y/2, +len.y/2);
            pos.z = common::getRandomDouble( 0.01, 1.00) + 0.5 * len.z;

            color = common::getRandomDouble(0.3, 0.7);

            obstacles.create_box("stone"+std::to_string(i*maxj+j), pos, len, .0, constants::materials::rock, color, friction);
        }

    obstacles.create_box("bar"   , {.0, -3.520, 0.10}, {1.50, 0.02, 0.02}, .0, constants::materials::rock  , colors::gray  , 10.0);
    obstacles.create_box("seesaw", {.0, -3.500, 0.20}, {1.50, 1.50, 0.01}, .0, constants::materials::normal, colors::white , 10.0);

}
