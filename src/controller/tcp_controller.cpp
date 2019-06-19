#include <controller/tcp_controller.h>

bool TCPController::establishConnection(int port)
{
    dsPrint("TCP Controller: listening to port %d...\n", port);
    fflush(stdout);
    socketServer = new SocketServer(port);
    if (socketServer->establish_connection())
    {
        dsPrint("Connection to client established.\nSending the robot's configuration to client.\n");
        TCPController::send_robot_configuration();
        return wait_for_ack();
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

    bool reload_model = false;

    /* reset frame record flag */
    config.record_frames = false;

    /* send message to client */
    if (not paused)
        send_ordered_info(time);

    paused = false; // client must continuously send pause signal
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
        if (starts_with(msg, "RESET"  )) { playSnapshot(robot, obstacles, &s1_init); reset(); continue; }

        /* save and restore snapshots */
        if (starts_with(msg, "SAVE"   )) { dsPrint("Saving state.\n"); recordSnapshot(robot, obstacles, &s2_user); continue; }
        if (starts_with(msg, "RESTORE")) { playSnapshot  (robot, obstacles, &s2_user); continue; }
        if (starts_with(msg, "NEWTIME")) { if (reset_time) reset_time(); continue; }

        /* simulator commands */
        if (starts_with(msg, "RECORD" )) { config.record_frames = true; continue; }
        if (starts_with(msg, "PAUSE"  )) { paused = true; continue; }
        if (starts_with(msg, "DONE"   )) { done = true; continue; }
        if (starts_with(msg, "EXIT"   )) { dsPrint("Received 'EXIT' command.\n"); return false; }

        /* model updates */
        if (starts_with(msg, "MODEL"  )) { reload_model = parse_update_model_command(msg.c_str()); continue; }
        if (starts_with(msg, "MOTOR"  )) { parse_update_motor_model(msg.c_str()); continue; }

        /* sensor quality */
        if (starts_with(msg, "SENSORS POOR")) { dsPrint("Setting poor sensor quality.\n"); low_quality_sensors = true;  continue; }
        if (starts_with(msg, "SENSORS GOOD")) { dsPrint("Setting good sensor quality.\n"); low_quality_sensors = false; continue; }

        /* misc */
        if (starts_with(msg, "FIXED")) { parse_toggle_fixed(msg.c_str()); continue; }
        if (starts_with(msg, "DESCRIPTION")) { send_robot_description_str(); continue; }

        /* error */
        if (fail_counter++ >= 42) { dsPrint("Too many messages without a 'DONE'-command.\n"); return false; }

        dsPrint("ERROR: unknown command: '%s'\n", msg.c_str());
    }

    if (reload_model) {
        reset();
        recordSnapshot(robot, obstacles, &s1_init);
        recordSnapshot(robot, obstacles, &s2_user);
        camera.set_viewpoint(robot.get_camera_center_obj(), robot.get_camera_setup());
        send_robot_configuration();
        wait_for_ack();
    }

    if (not paused)
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

    /* angular position */
    for (std::size_t i = 0; i < robot.number_of_joints(); ++i)
    {
        snprintf(tmp, buffer_size, "%lf ", robot.joints[i].get_low_resolution_position(low_quality_sensors));
        message.append(tmp);
    }

    /* angular velocity */
    for (std::size_t i = 0; i < robot.number_of_joints(); ++i)
    {
        snprintf(tmp, buffer_size, "%lf ", robot.joints[i].get_low_resolution_velocity(low_quality_sensors));
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

    /* transmit body names */
    for (std::size_t i = 0; i < robot.number_of_bodies(); ++i) {
        snprintf(tmp, buffer_size, "%lu %s\n", i, robot.bodies[i].name.c_str());
        message.append(tmp);
    }

    /* send message to socket */
    if (!socketServer->send_message(message))
        dsError("Could not send robot configuration message to client.\n");
}

bool TCPController::wait_for_ack(void)
{
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

std::vector<double> read_params(const char* msg, int* offset, unsigned num_params)
{
    std::vector<double> params;
    double p = 0.;

    if (num_params > 0)
    {
        params.reserve(num_params);
        for (unsigned int idx = 0; idx < num_params; ++idx)
        {
            if (1 == sscanf(msg, " %lf%n", &p, offset)) {
                msg += (*offset);
                params.emplace_back(p);
            } else {
                dsPrint("ERROR: bad parameter format for 'MODEL' : '%s'\n", msg);
                return {};
            }
        }
    }
    return params;
}

bool TCPController::parse_update_model_command(const char* msg)
{
    int offset = 5;
    msg += offset;

    int new_model_id;
    unsigned num_params;

    if (2 != sscanf(msg, " %d %u%n", &new_model_id, &num_params, &offset)) {
        dsPrint("ERROR: bad 'MODEL' format: '%s'\n", msg);
        return false;
    }

    msg += offset;
    auto params = read_params(msg, &offset, num_params);

    playSnapshot(robot, obstacles, &s1_init); // restore initial state

    robot.destroy();
    obstacles.destroy();
    landscape.destroy();
    Bioloid::create_robot(robot, new_model_id, params);
    Bioloid::create_scene(obstacles, landscape);
    return true;
}


void TCPController::parse_update_motor_model(const char* msg) {
    int offset = 5;
    msg += offset;

    unsigned num_params;

    if (1 != sscanf(msg, " %u%n", &num_params, &offset)) {
        dsPrint("ERROR: bad 'MOTOR' format: '%s'\n", msg);
        return;
    }

    msg += offset;
    auto params = read_params(msg, &offset, num_params);

    dsPrint("Reinitializing actuator model with %u parameters.\n", num_params);
    for (unsigned int idx = 0; idx < robot.number_of_joints(); ++idx)
        robot.joints[idx].reinit_motormodel(ActuatorParameters(params));

    return;
}


void TCPController::parse_toggle_fixed(const char* msg)
{
    unsigned int idx = 0;

    if (sscanf(msg, "FIXED %u", &idx) == 1)
    {
        if (idx < robot.number_of_bodies())
            robot.bodies[idx].toggle_fixed(universe.world);
        else
            dsPrint("ERROR: value #1 out of range (0...%u): '%s'\n", robot.number_of_bodies() - 1, msg);
    }
    else dsPrint("ERROR: bad 'FIXED' format: '%s'\n", msg);
}


void TCPController::send_robot_description_str()
{
    std::string message = robot.description;

    assert(!message.empty());

    if (message.back() != '\n')
        message.append("\n");

    dsPrint("Robot description requested.\n");

    /* send message to socket */
    if (!socketServer->send_message(message))
        dsError("Could not send robot description message to client.\n");
}

/* fin */

