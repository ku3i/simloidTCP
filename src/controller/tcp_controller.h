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
#include <sensors/accelsensor.h>

extern Configuration global_conf;

class TCPController : public Controller {
public:
    TCPController(Configuration& config, const physics& universe, Robot& robot, const Obstacle& obstacles, void (*r)())
    : Controller(universe, robot, obstacles, r)
    , config(config)
    {
        dsPrint("Starting TCP controller...");
        if (robot.number_of_joints() < 1)
            dsError("Bad Robot definition (%d joints).\n", robot.number_of_joints());
        dsPrint("done.\n");

        dsPrint("Recording initial snapshot.\n");
        recordSnapshot(robot, obstacles, &s1);
        recordSnapshot(robot, obstacles, &s2);
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

    void execute_controller();

    Snapshot s1;
    Snapshot s2;

    void send_ordered_info(const double time);
    void send_robot_configuration();

    Configuration& config;
};


#endif
