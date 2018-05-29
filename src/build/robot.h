#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <vector>
#include <string>

#include <basic/common.h>
#include <build/physics.h>
#include <build/joints.h>
#include <build/bodies.h>
#include <sensors/accelsensor.h>
#include <misc/camera.h>

struct ModelID {
    unsigned identifier;
    unsigned instance_no;

    friend bool operator==(ModelID const& lhs, ModelID const& rhs) {
        return lhs.identifier  == rhs.identifier
           and lhs.instance_no == rhs.instance_no;
    }

};



class Robot {
public:
    Robot(const dWorldID &world, const dSpaceID &space)
    : world(world)
    , space(space)
    , bodies(world, space, constants::max_bodies)
    , joints(constants::max_joints)
    , accels(constants::max_accels)
    , cam_center_obj(0)
    , cam_setup(Vector3(0.3,-0.3,0.3), 130.,-18.,0.)
    , model_id()
    { }
    const dWorldID&  world;
    const dSpaceID&  space;

    SolidVector bodies; // body parts
    JointVector joints; // joints
    AccelVector accels; // acceleration sensors

    std::size_t number_of_joints() const { return joints.get_size(); }
    std::size_t number_of_bodies() const { return bodies.get_size(); }
    std::size_t number_of_accels() const { return accels.get_size(); }

    void create_box( const std::string name
                   , const Vector3 pos
                   , const Vector3 len
                   , const double mass
                   , const double density
                   , const Color4 color
                   , const bool collision
                   , const double friction = dInfinity);
    void create_box( const std::string name
                   , const double posx, const double posy, const double posz
                   , const double lenx, const double leny, const double lenz
                   , const double mass
                   , const double density
                   , const Color4 color
                   , const bool collision
                   , const double friction = dInfinity);

    void create_segment( const std::string name
                       , const double posx, const double posy, const double posz
                       , const unsigned int direction
                       , const double length, const double radius
                       , const double mass
                       , const double density
                       , const Color4 color
                       , const bool collision
                       , const double friction = dInfinity);

    void attach_accel_sensor(std::string name_body);

    void connect_joint (
                    const std::string bodyname1, const std::string bodyname2,
                    const double relx, const double rely, const double relz,
                    const char axis,
                    const double jointstopLo_deg, const double jointstopHi_deg, const double jointposDefault_deg,
                    const JointType Type,
                    const std::string Name,
                    const std::string SymName = "",
                    const unsigned int torque_factor = 5
                );

    void print_statistics(void) const;

    void set_camera_center_on(std::string name_body);
    void set_camera_center_obj(unsigned int id) { if (id < number_of_bodies()) cam_center_obj = id; }

    dBodyID get_camera_center_obj(void)         const { return bodies[cam_center_obj].body; };
    unsigned int get_camera_center_obj_id(void) const { return cam_center_obj; }
    Camera_Setup get_camera_setup(void)         const { return cam_setup; }

    void setup_camera(Vector3 pos, double height, double pitch, double roll);

    void set_camera_center_on_next_obj();
    void set_camera_center_on_prev_obj();

    ~Robot() { dsPrint("Destroying robot.\n"); }

    ModelID const& get_model_id() const { return model_id; }

    void draw(Configuration const& conf)
    {
        /* draw robot's bodies */
        for (std::size_t i = 0; i < number_of_bodies(); ++i)
            bodies[i].draw(conf.show_aabb);

        /* draw robot's joints */
        if (conf.show_joints)
            for (std::size_t i = 0; i < number_of_joints(); ++i)
                joints[i].draw();

        /* draw robot's acceleration sensors */
        if (conf.show_accels)
            for (std::size_t i = 0; i < number_of_accels(); ++i)
                accels[i].draw();
    }

    void destroy(void) {
        bodies.destroy();
        joints.destroy();
        accels.destroy();
    }

private:
    unsigned int cam_center_obj;
    Camera_Setup cam_setup;

    ModelID model_id;
};


#endif // ROBOT_H_INCLUDED
