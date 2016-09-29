#include <controller/tcp_controller.h>

bool TCPController::establishConnection(int port)
{
    dsPrint("TCPController: listening to port %d...\n", port);
    fflush(stdout);
    socketServer = new SocketServer(port);
    if (socketServer->establish_connection())
    {
        dsPrint("Connection to client established.\nSending the robot's configuration to client.\n");
        TCPController::send_robot_configuration();
        dsPrint("Waiting for acknowledge.\n");
        std::string ack = socketServer->getNextLine();

        if (ack.compare(0, 3, "ACK") == 0)
            dsPrint("Acknowledge for configuration received.\n");
        else {
            dsPrint("Failed to receive acknowledge.\n");
            return false;
        }
        return true;
    }
    else
    {
        dsPrint("Failed to establish connection to client.\n");
        return false;
    }
}

inline bool starts_with(std::string msg, const char c_str[])
{
    return (msg.compare(0, strlen(c_str), c_str) == 0);
}

bool TCPController::control(const double time)
{
    bool done = false;
    unsigned int fail_counter = 0;
    std::string msg;

    /* send message to client */
    send_ordered_info(time);

    while (!done)
    {
        /* listen to socket */
        msg = socketServer->getNextLine();

        /* parse voltage commands */
        if (starts_with(msg, "UX ")) { parse_voltage_UX(msg.c_str()); continue; } // UX <voltage_0> <voltage_1> ... <voltage_N-1>
        if (starts_with(msg, "UA ")) { parse_voltage_UA(msg.c_str()); continue; } // UA <voltage>
        if (starts_with(msg, "UI ")) { parse_voltage_UI(msg.c_str()); continue; } // UI <joint_ID> <voltage>

        /* setpoint for PID controller */
        if (starts_with(msg, "PX ")) { parse_pidctrl_PX(msg.c_str()); continue; } // PX <position_0> <position_1> ... <position_N-1>
        if (starts_with(msg, "PA ")) { parse_pidctrl_PA(msg.c_str()); continue; } // PA <position>
        if (starts_with(msg, "PI ")) { parse_pidctrl_PI(msg.c_str()); continue; } // PI <joint_ID> <position>

        /* max. torque for PID */
        if (starts_with(msg, "TX ")) { parse_maxtorq_TX(msg.c_str()); continue; } // TX <maxtorque_0> <maxtorque_1> ... <maxtorque_N-1>
        if (starts_with(msg, "TA ")) { parse_maxtorq_TA(msg.c_str()); continue; } // TA <maxtorque>
        if (starts_with(msg, "TI ")) { parse_maxtorq_TI(msg.c_str()); continue; } // TI <joint_ID> <maxtorque>

        /* add impulse to body */
        if (starts_with(msg, "FX ")) { parse_impulse_FX(msg.c_str()); continue; } // FX <force_x_0> <force_y_0> <force_z_0> ... <force_x_N-1> <force_y_N-1> <force_z_N-1>
        if (starts_with(msg, "FA ")) { parse_impulse_FA(msg.c_str()); continue; } // FA <force_x> <force_y> <force_z>
        if (starts_with(msg, "FI ")) { parse_impulse_FI(msg.c_str()); continue; } // FI <body_ID> <force_x> <force_y> <force_z>

        /* gravity */
        if (starts_with(msg, "GRAVITY ON" )) { universe.set_gravity(true);  continue; }
        if (starts_with(msg, "GRAVITY OFF")) { universe.set_gravity(false); continue; }

        /* reset */
        if (starts_with(msg, "RESET"  )) { playSnapshot(robot, obstacles, &s1); reset(); continue; }

        /* save and restore snapshots */
        if (starts_with(msg, "SAVE"   )) { recordSnapshot(robot, obstacles, &s2); continue; }
        if (starts_with(msg, "RESTORE")) { playSnapshot  (robot, obstacles, &s2); continue; }

        if (starts_with(msg, "NEWTIME")) { if (reset_time) reset_time(); continue; }

        /* simulator commands */
        if (starts_with(msg, "DONE")) { done = true; continue; }
        if (starts_with(msg, "EXIT")) { dsPrint("Received 'EXIT' command.\n"); return false; }

        /* error */
        if (fail_counter++ >= 42) { dsPrint("Too many messages without a 'DONE'-command.\n"); return false; }

        dsPrint("ERROR: unknown command: '%s'\n", msg.c_str());
    }

    execute_controller();
    return true;
}

void TCPController::send_ordered_info(const double time)
{
    const std::size_t buffer_size = 4096;
    std::string message;
    char tmp[buffer_size];

    // time stamp
    snprintf(tmp, buffer_size, "%lf ", time);
    message.append(tmp);

    /* anglular position */
    for (std::size_t i = 0; i < robot.number_of_joints(); ++i)
    {
        snprintf(tmp, buffer_size, "%lf ", robot.joints[i].get_low_resolution_position());
        message.append(tmp);
    }

    /* angular velocity */
    for (std::size_t i = 0; i < robot.number_of_joints(); ++i)
    {
        snprintf(tmp, buffer_size, "%lf ", robot.joints[i].get_low_resolution_velocity());
        message.append(tmp);
    }

    /* acceleration */
    for (std::size_t i = 0; i < robot.number_of_accels(); ++i)
    {
        const Vector3& acc = robot.accels[i].update();
        snprintf(tmp, buffer_size, "%lf %lf %lf ", acc.x, acc.y, acc.z);
        message.append(tmp);
    }

    /* body locations + velocities */
    for (std::size_t i = 0; i < robot.number_of_bodies(); ++i)
    {
        const Vector3& pos = robot.bodies[i].get_position();
        const Vector3& vel = robot.bodies[i].get_velocity();
        snprintf( tmp, buffer_size, "%lf %lf %lf %lf %lf %lf "
                , pos.x, pos.y, pos.z, vel.x, vel.y, vel.z );
        message.append(tmp);
    }

    /**TODO:
     * think about how we could get out of the simulator the current power,
     * Is it sufficient to only provide a sum power value, or should we provide it for each joint?
     * Shall we instead of power provide the motor current?
     */

    /* send message to socket */
    if (!socketServer->send_message(message))
        dsError("Could not send ordered info message to client!\n");
}

void TCPController::send_robot_configuration()
{
    const std::size_t buffer_size = 4096;
    std::string message;
    char tmp[buffer_size];

    snprintf(tmp, buffer_size, "%lu %lu %lu\n", robot.number_of_bodies(), robot.number_of_joints(), robot.number_of_accels());
    message.append(tmp);

    for (std::size_t i = 0; i < robot.number_of_joints(); ++i)
    {
        snprintf(tmp, buffer_size, "%lu %u %u %e %e %e %s\n",
            i,
            robot.joints[i].type,
            robot.joints[i].symmetric_joint,
            robot.joints[i].stop_lo,
            robot.joints[i].stop_hi,
            robot.joints[i].position_default,
            robot.joints[i].name.c_str());
        message.append(tmp);
    }

    /* send message to socket */
    if (!socketServer->send_message(message))
        dsError("Could not send robot configuration message to client.\n");
}

void TCPController::execute_controller()
{
    robot.joints.apply_control_all();
}

void TCPController::reset()
{
    robot.joints.reset_all();
    robot.accels.reset_all();
    if (reset_time) reset_time();
}

void TCPController::parse_pidctrl_PA(const char* msg)
{
    double value = 0.0;

    if (1 == sscanf(msg, "PA %lf", &value))
    {
        for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
             robot.joints[idx].set_position(value);
    }
    else dsPrint("ERROR: bad 'PA' format: '%s'\n", msg);
}

void TCPController::parse_pidctrl_PX(const char* msg)
{
    int offset = 2;
    double value = 0.0;

    msg += offset;
    for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
    {
        if (1 == sscanf(msg, " %lf%n", &value, &offset))
        {
            msg += offset;
             robot.joints[idx].set_position(value);
        }
        else {
            dsPrint("ERROR: bad 'PX' format: '%s'\n", msg);
            break;
        }
    }
}

void TCPController::parse_pidctrl_PI(const char* msg)
{
    unsigned int idx = 0;
    double value = 0.0;

    if (2 == sscanf(msg, "PI %u %lf", &idx, &value))
    {
        if (idx < robot.number_of_joints())
            robot.joints[idx].set_position(value);
        else
            dsPrint("ERROR: joint value out of range (0...%u): '%s'\n", robot.number_of_joints() - 1, msg);
    }
    else dsPrint("ERROR: bad 'PI' format: '%s'\n", msg);
}

void TCPController::parse_maxtorq_TA(const char* msg)
{
    double value = 0.0;

    if (1 == sscanf(msg, "TA %lf", &value))
    {
        if ((value >= 0) && (value <= 1.0))
        {
            for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
                robot.joints[idx].set_pidmaxtorque(value);
        }
        else dsPrint("ERROR: value out of range (0...+1): '%s'\n", msg);
    }
    else dsPrint("ERROR: bad 'TA' format: '%s'\n", msg);
}

void TCPController::parse_maxtorq_TX(const char* msg)
{
    unsigned int offset = 2;
    double value = 0.0;

    msg += offset;
    for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
    {
        if (1 == sscanf(msg, " %lf%n", &value, &offset))
        {
            msg += offset;
            robot.joints[idx].set_pidmaxtorque(value);
        }
        else {
            dsPrint("ERROR: bad 'TX' format: '%s'\n", msg);
            break;
        }
    }
}

void TCPController::parse_maxtorq_TI(const char* msg)
{
    unsigned int idx = 0;
    double value = 0.0;

    if (2 == sscanf(msg, "TI %u %lf", &idx, &value))
    {
        if (idx < robot.number_of_joints())
            robot.joints[idx].set_pidmaxtorque(value);
        else
            dsPrint("ERROR: joint number out of range (0...%u): '%s'\n", robot.number_of_joints() - 1, msg);

    }
    else dsPrint("ERROR: bad 'TI' format: '%s'\n", msg);
}

void TCPController::parse_voltage_UA(const char* msg)
{
    double value = 0.0;
    if (1 == sscanf(msg, "UA %lf", &value))
    {
        for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
            robot.joints[idx].set_voltage(value);
    }
    else dsPrint("ERROR: bad 'UA' format: '%s'\n", msg);
}

void TCPController::parse_voltage_UX(const char* msg)
{
    int offset = 2;
    double value = 0.0;

    msg += offset;
    for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
    {
        if (1 == sscanf(msg, " %lf%n", &value, &offset)) {
            msg += offset;
            robot.joints[idx].set_voltage(value);
        } else {
            dsPrint("ERROR: bad 'UX' format: '%s'\n", msg);
            break;
        }
    }
}

void TCPController::parse_voltage_UI(const char* msg)
{
    unsigned int idx = 0;
    double value = 0.0;

    if (2 == sscanf(msg, "UI %u %lf", &idx, &value))
    {
        if (idx < robot.number_of_joints())
            robot.joints[idx].set_voltage(value);
        else
            dsPrint("ERROR: value #1 out of range (0...%u): '%s'\n", robot.number_of_joints() - 1, msg);
    }
    else dsPrint("ERROR: bad 'UI' format: '%s'\n", msg);
}

void TCPController::parse_impulse_FA(const char* msg)
{
    Vector3 force(0.0);
    if (3 == sscanf(msg, "FA %lf %lf %lf", &force.x, &force.y, &force.z))
    {
        for (unsigned int idx = 0; idx < robot.number_of_bodies(); ++idx)
            robot.bodies[idx].set_impulse(force);
    }
    else dsPrint("ERROR: bad 'FA' format: '%s'\n", msg);
}

void TCPController::parse_impulse_FX(const char* msg)
{
    int offset = 2;
    Vector3 force(0.0);

    msg += offset;
    for (unsigned int idx = 0; idx < robot.number_of_bodies(); ++idx)
    {
        if (3 == sscanf(msg, " %lf %lf %lf%n", &force.x, &force.y, &force.z, &offset)) {
            msg += offset;
            robot.bodies[idx].set_impulse(force);
        } else {
            dsPrint("ERROR: bad 'FX' format: '%s'\n", msg);
            break;
        }
    }
}

void TCPController::parse_impulse_FI(const char* msg)
{
    unsigned int idx = 0;
    Vector3 force(0.0);

    if (sscanf(msg, "FI %u %lf %lf %lf", &idx, &force.x, &force.y, &force.z) == 4)
    {
        if (idx < robot.number_of_bodies())
            robot.bodies[idx].set_impulse(force);
        else
            dsPrint("ERROR: value #1 out of range (0...%u): '%s'\n", robot.number_of_bodies() - 1, msg);
    }
    else dsPrint("ERROR: bad 'FI' format: '%s'\n", msg);
}

/* fin */

