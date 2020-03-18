/*--------------------------------------------------------------+
 | simloidTCP                                                   |
 |                                                              |
 | Simulation Environment                                       |
 |                                                              |
 | Authors: Daniel Hein      dhein(at)informatik.hu-berlin.de   |
 |          Matthias Kubisch kubisch(at)informatik.hu-berlin.de |
 |                                                              |
 | Humboldt-Universitaet zu Berlin                              |
 | Labor fuer Neurorobotik                                      |
 +--------------------------------------------------------------*/

#include <ode/ode.h>
#include <math.h>
#include <iostream>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#include <cassert>

#include <draw/drawstuff.h>

#include <basic/version.h>
#include <basic/common.h>
#include <basic/constants.h>
#include <basic/unitime.h>
#include <basic/configuration.h>
#include <basic/signals.h>
#include <basic/snapshot.h>

#include <controller/tcp_controller.h>

#include <build/bioloid.h>
#include <build/heightfield.h>
#include <build/robot.h>
#include <build/physics.h>
#include <build/bodies.h>
#include <build/obstacles.h>

#include <misc/camera.h>

/* dynamics and objects */
static physics     universe;
static Robot       robot(universe.world, universe.space);
static Obstacle    obstacles(universe.world, universe.space);
static Landscape   landscape(universe.space);
static Camera      camera;

/* time and snapshots */
static double simtime;
static Snapshot s1;
static Snapshot s2;

/* main loop */
static bool continueLoop = true;

/* controller */
static Controller* controller;

/* timer */
static double  vel = 0.0;
static double  intervalSimTime = 0.0;
static UniTime intervalBeginRealTime = UniTime::getTimeStamp();
static UniTime uniTimeStepLength;
static UniTime lastDrawFrameTime;
static UniTime duration = UniTime(0, 0);

/* status FPS */
static double frameTime;
static int intervalFrames = 0;

static double bvel;

/* Configuration */
Configuration global_conf = Configuration();


/* start simulation - set viewpoint */
static void start(void)
{
    if (!global_conf.disable_graphics) {
        camera.set_viewpoint(robot.get_camera_center_obj(), robot.get_camera_setup());

        dsPrint("Program controls:\n"
                "   1: toggle disable/enable graphics\n"
                "   2: reset physics\n"
                "   3: save snapshot of physics\n"
                "   4: restore snapshot of physics\n"
                "   R: toggle realtime on/off\n\n"
               );
    }
    uniTimeStepLength = UniTime(0, (int)(global_conf.step_length*1000000000));

    frameTime = 1.0 / global_conf.fps;
    if (frameTime > 0.5) {
        frameTime = 0.5;
        dsPrint("Min. fps: 2 => setting fps to 2\n");
    }


    dsPrint( "Starting simulation with step=%.3lfs and fps=%.0lf/s%s\n"
           , global_conf.step_length, 1.0/frameTime, global_conf.use_fps_control ? "" : " (fps ctrl off, showing each step)");

    lastDrawFrameTime = UniTime(0,0);
}

/* stop simulation */
static void stop() { dsPrint("Exiting simulation.\n"); }

void reset_time(void) { simtime = 0.0; }

static void reset_simulator(void)
{
    playSnapshot(robot, obstacles, &s1);
    controller->reset();
    reset_time();
}

/* called when a key is being pressed */
static void command(int cmd, bool /*shift*/)
{
    switch (cmd)
    {
        /* camera */
        case 'a': camera.turn_cw ();                                              break;
        case 'd': camera.turn_ccw();                                              break;
        case 'w': camera.zoom_in ();                                              break;
        case 's': camera.zoom_out();                                              break;
        case 't': camera.toggle_rotate();                                         break;
        case 'f': camera.toggle_follow(robot.get_camera_center_obj());            break;
        case '+': robot.set_camera_center_on_next_obj();                          break;
        case '-': robot.set_camera_center_on_prev_obj();                          break;

        /* reset */
        case '2': reset_simulator();                                              break;

        /* snapshots */
        case '3': recordSnapshot(robot, obstacles, &s2);                          break;
        case '4': playSnapshot  (robot, obstacles, &s2);                          break;

        /* drawing */
        case '1': global_conf.draw_scene        = !global_conf.draw_scene;        break;
        case 'r': global_conf.real_time         = !global_conf.real_time;         break;
        case 'm': global_conf.use_fps_control   = !global_conf.use_fps_control;   break;
        case 'z': global_conf.show_time_stat    = !global_conf.show_time_stat;    break;
        case 'j': global_conf.show_joints       = !global_conf.show_joints;       break;
        case 'k': global_conf.show_contacts     = !global_conf.show_contacts;     break;
        case 'l': global_conf.show_accels       = !global_conf.show_accels;       break;
        case 'c': global_conf.show_cam_position = !global_conf.show_cam_position; break;
    } // switch
}

void
print_time_statistics(void)
{
    if (global_conf.draw_scene && !global_conf.disable_graphics)
    {
        const float pos[2] = {-0.98, 0.94};
        auto const& rp = -robot.bodies[0].get_position().y;

        glprintf( pos[0], pos[1], 0, 0.02
                , "time: %5.2lf  sim: %.2lfx %s  fps: %.2lf%s  walking: v=%.2lf m/s  d=%5.2f m"
                , simtime, vel, (global_conf.real_time?"[real]":"      ")
                , global_conf.fps, global_conf.use_fps_control? " ":"!", bvel, rp);

        if (global_conf.show_cam_position)
        {
            float vpos[3], vdir[3];
            camera.get_viewpoint(vpos, vdir);

            glprintf( pos[0], pos[1]-0.05, 0, /*size=*/0.02
                    , "cam: (%.2f, %.2f, %.2f : %.1f, %.1f, %.1f)"
                    , vpos[0], vpos[1], vpos[2], vdir[0], vdir[1], vdir[2] );
        }
    }
}

static void draw_robot_and_scene()
{
    /* draw height field */
    for (unsigned int i = 0; i < landscape.number_of_heightfields(); ++i)
        landscape.heightfields[i].draw();

    /* draw scene objects and obstacles */
    for (unsigned int i = 0; i < obstacles.number_of_objects(); ++i)
        obstacles.objects[i].draw(false);

    /* draw the robot */
    robot.draw(global_conf);

    /* draw time and velocity information */
    if (global_conf.show_time_stat) print_time_statistics();

    /* update camera center of rotation, follow etc. */
    camera.update(robot.get_camera_center_obj());
}


static void physics_step(void) {
    dSpaceCollide(universe.space, &universe, &near_callback);  // collision detection
    dWorldStep(universe.world, global_conf.step_length);       // world simulation step
    dJointGroupEmpty(universe.contactgroup);                   // remove all contact joints

    simtime         += global_conf.step_length;                // increase time
    intervalSimTime += global_conf.step_length;
    //printf("t: %5.2f\n", simtime);
}


static void sim_loop(int pause, int singlestep) /**TODO this function is far too long, split up!*/
{
    static UniTime ref_time = UniTime::getTimeStamp();

    UniTime current_time = UniTime{};

    assert(continueLoop);

    do {
        const UniTime start_time = UniTime::getTimeStamp();

        /* physics */
        if (not (pause or controller->is_paused()))
            physics_step();

        /* controller */
        if (not pause && (controller != nullptr))
            continueLoop = controller->control(simtime);

        /* Timer */
        if (      global_conf.draw_scene
             and !global_conf.disable_graphics
             and  global_conf.real_time
             and !(pause or controller->is_paused())
           )
        {
            current_time = UniTime::getTimeStamp();
            ref_time = ref_time + uniTimeStepLength;

            if (current_time < ref_time) { /* we still have time, so wait until timed out */
                struct timeval tv{0, (ref_time - current_time).usec};
                select(1, NULL, NULL, NULL, /*timeout=*/&tv);
            }
            else { /* pass now */
                if ((current_time - ref_time).sec > 1) {
                    ref_time = UniTime::getTimeStamp();
                }
            }
        }

        current_time = UniTime::getTimeStamp();
        duration = duration * 0.7 + (current_time - start_time) * 0.3;

        if (pause) usleep(1000);

    } while (     continueLoop
             and !singlestep
             and global_conf.use_fps_control
             and ((double)((current_time - lastDrawFrameTime + duration * 0.5).usec) / 1000000.0) < frameTime
            );

    current_time = UniTime::getTimeStamp();
    lastDrawFrameTime = current_time;

    /* refresh velocity and fps */
    if ((current_time - intervalBeginRealTime).sec)
    {
        const double intervalTime = (current_time - intervalBeginRealTime).fseconds();

        vel = intervalSimTime / intervalTime;
        bvel = (intervalSimTime > 0.0001) ? common::dist2D((const double *) dBodyGetPosition(robot.get_camera_center_obj()), camera.lastPos) / intervalSimTime
                                          : 0.0;

        camera.lastPos[0] = dBodyGetPosition(robot.get_camera_center_obj())[0];
        camera.lastPos[1] = dBodyGetPosition(robot.get_camera_center_obj())[1];
        camera.lastPos[2] = dBodyGetPosition(robot.get_camera_center_obj())[2];

        intervalSimTime = 0.0;

        global_conf.fps = (double)intervalFrames / intervalTime; /**What??*/
        intervalFrames = 0;

        intervalBeginRealTime = current_time;
    }

    /* drawing robot and scene objects */
    if (global_conf.draw_scene && !global_conf.disable_graphics)
        draw_robot_and_scene();

    ++intervalFrames;
    if (not continueLoop) dsPrint("Leaving simulation loop. Shutting down simloid.\n");
}


void
print_programm_info(std::string executable_name = "")
{
    dsPrint( "%s Simulation Environment\n"
             "   Report bugs to kubisch@informatik.hu-berlin.de\n\n"
           , version::name.c_str() );

    if (executable_name != "")
        dsPrint("Help: %s --help\n\n", executable_name.c_str());
}


void
print_help(void)
{
    print_programm_info();

    std::cout << "Help:\n"
              << "   --configfile | -f               - name of config file\n"
              << "   --nographics | -ng              - run without x-window\n"
              << "   --norealtime | -nr              - run with maximal speed"
              << "   --help | -h                     - print this message\n"
              << "   --notex                         - no textures\n"
              << "   --noshadow(s)                   - no shadows\n"
              << "   --port <port> | -p <port>       - use tcp port number\n"
              << "   --steplength <time> | -s <time> - length of one simstep in sec\n"
              << "   --fps [<fps>|off]               - frames per second in 1/sec or 'off'\n"
              << "                                     'off' means, each simstep is drawn\n"
              << "   --pause                         - initial pause\n\n";
}


/**This should be re-factored next*/
void
readOptions (int argc, char* argv[])
{
    // override standard options from simloid.conf if submitted by -f
    for (int i = 1; i < argc; i++)
    {
        if ((strncmp(argv[i], "--configfile", 12) == 0) || (strncmp(argv[i], "-f", 2) == 0))
        {
            if (argc > i+1)
            {
                if (NULL != argv[i+1])
                {
                    dsPrint("Reading alternative config file %s...", argv[i+1]);
                    global_conf.readConfigurationFile(argv[i+1]);
                    dsPrint("done.\n");
                }
                else
                {
                    dsError("Invalid Argument.\n");
                }

            } else
            {
                dsError("Missing argument for file name option.\n");
            }
            break;
        }
    }


    for (int i = 1; i < argc; ++i)
    {
        if ((strncmp(argv[i], "--nographics", 12) == 0) || (strncmp(argv[i], "-ng", 3) == 0))
        {
            global_conf.disable_graphics = true;
        }
        else if ((strncmp(argv[i], "--norealtime", 12) == 0) || (strncmp(argv[i], "-nr", 3) == 0))
        {
            global_conf.real_time = false;
        }
        else if ((strncmp(argv[i], "--help", 6) == 0) || (strncmp(argv[i], "-h", 2) == 0))
        {
            print_help();
            exit(0);
        }
        else if ((strncmp(argv[i], "--port", 6) == 0) || (strncmp(argv[i], "-p", 2) == 0))
        {
            if (argc < i+2)
            {
                dsPrint("usage: %s -p <port>\n", argv[0]);
                exit(0);
            }
            else
            {
                global_conf.tcp_port = atoi(argv[i+1]);
                ++i;
            }
        }
        else if ((strncmp(argv[i], "--steplength", 12) == 0) || (strncmp(argv[i], "-s", 2) == 0))
        {
            if (argc < i+2)
            {
                dsPrint("usage: %s -s <steplength>\n", argv[0]);
                exit(0);
            }
            else
            {
                global_conf.step_length = atof(argv[i+1]);
                ++i;
            }
        }
        else if (strncmp(argv[i], "--fps", 5) == 0)
        {
            if (argc < i+2)
            {
                dsPrint("usage: %s --fps <fps>\n", argv[0]);
                exit(0);
            }
            else if (strncmp(argv[i+1], "off", 3) == 0)
            {
                global_conf.use_fps_control = false;
                ++i;
            }
            else
            {
                global_conf.fps = atof(argv[i+1]);
                ++i;
            }
        }
        else if (strncmp(argv[i], "--robot", 8) == 0)
        {
            if (argc < i+2)
            {
                dsPrint("usage: %s --robot <robot_index_no>\n", argv[0]);
                exit(0);
            }
            else
            {
                global_conf.robot = atoi(argv[i+1]);
                ++i;
            }
        }
        else if (strncmp(argv[i], "--scene", 8) == 0)
        {
            if (argc < i+2)
            {
                dsPrint("usage: %s --scene <scene_index_no>\n", argv[0]);
                exit(0);
            }
            else
            {
                global_conf.scene = atoi(argv[i+1]);
                ++i;
            }
        }

    }
}


void
sigtest(int sig)
{
    switch (sig) {
        case SIGTERM: dsPrint("Received SIGTERM.\n"); continueLoop = false; break;
        case SIGINT:  dsPrint("Received SIGINT. \n"); continueLoop = false; break;
        default:      dsPrint("Received unknown signal: %d\n", sig);
    }
}


int
main(int argc, char **argv)
{
    /* initialize random generator */
    srand(time(NULL));

    print_programm_info(argv[0]);

    global_conf.readConfigurationFile("simloid.conf");
    readOptions(argc, argv); // this may override some standard options from conf file

    /* set signal handler */
    Signals signal(sigtest);

    /* setup pointers to drawstuff callback functions */
    dsFunctions fn;
    fn.version          = DS_VERSION;
    fn.start            = &start;
    fn.step             = &sim_loop;
    fn.command          = &command;
    fn.stop             = &stop;
    fn.path_to_textures = "./textures";
    fn.drawScene        = &global_conf.draw_scene;
    fn.recordFrames     = &global_conf.record_frames;
    fn.disableGraphics  =  global_conf.disable_graphics;
    fn.continueLoop     = &continueLoop;

    global_conf.draw_scene = !global_conf.disable_graphics;
    /** so actually disable_graphics and draw_scene mean the same thing*/

    /* create Robot */
    Bioloid::create_robot(robot);
    Bioloid::create_scene(obstacles, landscape);

    /* create TCP Controller */
    controller = new TCPController(global_conf, universe, robot, obstacles, landscape, reset_time, camera);
    if (((TCPController*)controller)->establishConnection(global_conf.tcp_port))
    {
        /* run simulation */
        bool initial_pause = global_conf.initial_pause && !global_conf.disable_graphics;
        dsSimulationLoop(argc, argv, global_conf.window_width, global_conf.window_height, &fn, (int) initial_pause);
    }
    else dsError("Could not start TCP controller.\n");

    /* clean up simulation */
    delete controller;

    return 0;
}

/* fin */
