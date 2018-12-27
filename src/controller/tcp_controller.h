#ifndef _TCPCONTROLLER_H_
#define _TCPCONTROLLER_H_

#include <draw/drawstuff.h>
#include <controller/controller.h>
#include <communication/socketserver.h>
#include <basic/common.h>
#include <basic/configuration.h>
#include <basic/constants.h>
#include <basic/snapshot.h>
#include <build/robot.h>
#include <build/bioloid.h>
#include <sensors/accelsensor.h>
#include <misc/camera.h>

extern Configuration global_conf;

class TCPController : public Controller {
public:
    TCPController( Configuration& config
                 , physics const& universe
                 , Robot& robot
                 , Obstacle& obstacles
                 , Landscape& landscape
                 , void (*r)()
                 , Camera& camera )
    : Controller(universe, robot, obstacles, landscape, r)
    , config(config)
    , camera(camera)
    {
        dsPrint("Starting TCP controller...");
        if (robot.number_of_joints() < 1)
            dsError("Bad Robot definition (%d joints).\n", robot.number_of_joints());
        dsPrint("done.\n");

        dsPrint("Recording initial snapshot.\n");
        recordSnapshot(robot, obstacles, &s1_init);
        recordSnapshot(robot, obstacles, &s2_user);
    };

    ~TCPController() {
        delete socketServer;
    }

    bool control(const double time);
    bool establishConnection(int port);
    void reset();

private:
    SocketServer *socketServer;

    /* functions for command parsing */
    void parse_voltage_UA(const char* msg);
    void parse_voltage_UX(const char* msg);
    void parse_voltage_UI(const char* msg);

    void parse_pidctrl_PA(const char* msg);
    void parse_pidctrl_PX(const char* msg);
    void parse_pidctrl_PI(const char* msg);

    void parse_maxtorq_TA(const char* msg);
    void parse_maxtorq_TX(const char* msg);
    void parse_maxtorq_TI(const char* msg);

    void parse_impulse_FA(const char* msg);
    void parse_impulse_FX(const char* msg);
    void parse_impulse_FI(const char* msg);

    bool parse_update_model_command(const char* msg);
    void parse_update_motor_model(const char* msg);

    void execute_controller();

    Snapshot s1_init;
    Snapshot s2_user;

    void send_ordered_info(const double time);
    void send_robot_configuration(void);
    bool wait_for_ack(void);

    Configuration& config;
    Camera& camera;
};


#endif
