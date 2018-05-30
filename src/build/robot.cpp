#include <build/robot.h>

void check_joint_axis(const char axis);

void Robot::create_box(std::string name,
                       const Vector3 pos,
                       const Vector3 len,
                       const double mass, const double density,
                       const Color4 color,
                       bool collision, const double friction)
{
    bodies.create_box(name, pos, len, mass, density, color, collision, friction);
}

void Robot::create_box(const std::string name,
                       const double posx, const double posy, const double posz,
                       const double lenx, const double leny, const double lenz,
                       const double mass, const double density,
                       const Color4 color,
                       const bool collision, const double friction)
{
    bodies.create_box(name, Vector3(posx,posy,posz), Vector3(lenx,leny,lenz), mass, density, color, collision, friction);
}



void Robot::create_segment( const std::string name
                          , const double posx, const double posy, const double posz
                          , const unsigned int direction
                          , const double length, const double radius
                          , const double mass
                          , const double density
                          , const Color4 color
                          , const bool collision
                          , const double friction)
{
    bodies.create_capsule(name, Vector3(posx,posy,posz), direction, length, radius, mass, density, color, collision, friction);
}


void Robot::attach_accel_sensor(const std::string bodyname, bool keep_original_color)
{
    unsigned int objnr = bodies.get_body_id_by_name(bodyname);
    if (objnr == bodies.get_size())
        dsError("No body with name '%s' to attach an accelerometer on.\n", bodyname.c_str());

    dsPrint("Attaching acceleration sensor to '%s' (%d)\n", bodyname.c_str(), objnr);
    accels.attach(bodies[objnr].body, _left, _forward, _up); // right hand rule with x,y,z
    if (!keep_original_color)
        bodies[objnr].color = colors::orange;
}

void Robot::connect_joint( const std::string bodyname1, const std::string bodyname2,
                           const double relx, const double rely, const double relz,
                           const char axis,
                           const double jointstopLo_deg, const double jointstopHi_deg, const double jointposDefault_deg,
                           const JointType Type,
                           const std::string Name, const std::string SymName,
                           double torque_factor,
                           ActuatorParameters const& conf )
{
    check_joint_axis(axis);

    double jointposDefault = common::deg2rad(jointposDefault_deg);
    double jointstopHi     = common::deg2rad(jointstopHi_deg    );
    double jointstopLo     = common::deg2rad(jointstopLo_deg    );

    // get the bodys' object IDs by name
    unsigned int body1 = bodies.get_body_id_by_name(bodyname1);
    unsigned int body2 = bodies.get_body_id_by_name(bodyname2);

    if (body1 == bodies.get_size()) { dsError("Cannot find such an object for connection: '%s'\n", bodyname1.c_str()); }
    if (body2 == bodies.get_size()) { dsError("Cannot find such an object for connection: '%s'\n", bodyname2.c_str()); }

    unsigned int joint_id = joints.create(world, bodies, body1, body2, Type, Name, jointstopLo, jointstopHi, jointposDefault, Vector3(relx, rely, relz), axis, torque_factor, conf);

    bool result = joints.add_symmetric(joint_id, SymName);
    if (not result)
        dsPrint("Did not found yet symmetric joint for %02u\n", joint_id);
}


void Robot::print_statistics(void) const
{
    dsPrint("Robot statistics:\n   Bodies: %lu\n   Joints: %lu\n   Accels: %lu\n   Weight: %.3lf kg\n   CoM: (%.2lf, %.2lf, %.2lf)\n\n"
           , number_of_bodies()
           , number_of_joints()
           , number_of_accels()
           , bodies.get_total_mass().mass
           , bodies.get_total_mass().c[0]
           , bodies.get_total_mass().c[1]
           , bodies.get_total_mass().c[2] );
}

void Robot::set_camera_center_on(const std::string bodyname)
{
    unsigned int objnr = bodies.get_body_id_by_name(bodyname);
    if (objnr == bodies.get_size())
        dsError("No joint with name '%s' to focus.\n", bodyname.c_str());

    dsPrint("Setting camera focus on body '%s' (%d)\n", bodyname.c_str(), objnr);
    cam_center_obj = objnr;
}

void Robot::set_camera_center_on_next_obj(void)
{
    ++cam_center_obj;
    if (cam_center_obj == number_of_bodies())
        cam_center_obj = 0;

    dsPrint("Camera focus on: %u '%s'\n", cam_center_obj, bodies[cam_center_obj].name.c_str());
}

void Robot::set_camera_center_on_prev_obj(void)
{
    if (cam_center_obj > 0) --cam_center_obj;
    else cam_center_obj = number_of_bodies() - 1;

    dsPrint("Camera focus on: %u '%s'\n", cam_center_obj, bodies[cam_center_obj].name.c_str());
}


/* set and get the camera position. xyz is the cameria position (x,y,z).
 * hpr contains heading, pitch and roll numbers in degrees. heading=0
 * points along the x axis, pitch=0 is looking towards the horizon, and
 * roll 0 is "unrotated".
 */
void Robot::setup_camera(Vector3 pos, double heading, double pitch, double roll)
{
    cam_setup.pos     = pos;
    cam_setup.heading = heading;
    cam_setup.pitch   = pitch;
    cam_setup.roll    = roll;
    dsPrint("Setting camera to position(%1.2f,%1.2f,%1.2f) and hpr(%1.2f,%1.2f,%1.2f).\n",
        pos.x, pos.y, pos.z, heading, pitch, roll);
}

void check_joint_axis(const char axis)
{
    switch(axis)
    {
        case 'x': dsPrint("Created x axis (roll) \n"); break;
        case 'X': dsPrint("Created X axis (roll) \n"); break;
        case 'y': dsPrint("Created y axis (pitch)\n"); break;
        case 'Y': dsPrint("Created Y axis (pitch)\n"); break;
        case 'z': dsPrint("Created z axis (yaw)  \n"); break;
        case 'Z': dsPrint("Created Z axis (yaw)  \n"); break;
        default:  dsError("Unknown axis: %c.", axis);
    }
}
