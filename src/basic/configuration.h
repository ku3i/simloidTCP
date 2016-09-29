#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include <string>
#include <vector>
#include <iostream>

#include <basic/version.h>
#include <filehandler/file_handler.h>

/* TODO:
 * Uint
 * Parameter werden hier nicht auf Richtigkeit geprüft. */
class Configuration
{
public:
    Configuration();
    ~Configuration();

    bool readConfigurationFile(const char* filename);

    /* General */
    int    tcp_port;            // TCP Port für TCPController //TODO make to Uint

    /* Environment */
    int    robot;               // number of the robot's body plan //TODO make to string
    int    scene;               // number of the experimental setup (scene)
    bool   initial_gravity;     // Simulator mit Gravitation anschalten

    /* Simulation */
    double step_length;         // Schrittweite, mit der ein Simulationsschritt ausgefuehrt wird
    bool   real_time;           // Simulationsgeschwindigkeit auf 1.0x bremsen
    bool   initial_pause;       // Soll im Pause-Modus gestartet werden?
    double contact_soft_ERP;    // error reduction parameter during contacts
    double contact_soft_CFM;    // constraint force mixing during contacts

    /* Visualization */
    bool   disable_graphics;    // creating window? (otherwise just running on console)
    bool   draw_scene;          // drawing scene in window?
    bool   show_aabb;           // show geom AABBs?
    bool   show_contacts;       // show contact points?
    bool   show_joints;         // show the joint's anchor and axis
    bool   show_accels;         // show the acceleration sensors
    bool   show_cam_position;   // display the postion and the heading of the camera
    double fps;                 // Angestrebte Frames per second
    bool   use_fps_control;     // FPS-Controller benutzen oder nicht
    int    window_width;
    int    window_height;

    /* Controller */
    double init_max_torque;	    // default maxtorque used at pid controller
    double pidP;                // P-Value for PID-Controller
    double pidI;                // I-Value for PID-Controller
    double pidD;                // D-Value for PID-Controller


private:
    enum ValueType {
        UNKOWN,
        BOOL  ,
        INT   ,
        DOUBLE,
        STRING
    };
    struct parameter {
        std::string category;
        std::string name;
        void* variable;
        ValueType type;
        std::string description;
        parameter(const char* c, const char* n, void* v, ValueType t, const char* d) {
            category = std::string(c);
            name = std::string(n);
            variable = v;
            type = t;
            description = std::string(d);
        }
    };
    void init_parameter_vector();
    std::vector<parameter> theParameterVector;
    bool createConfigFile(const char* filename);
    std::vector<parameter>::const_iterator lookup(std::vector<parameter>& pv, const char* name);
};

#endif
