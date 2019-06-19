#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <vector>
#include <string>

#include <basic/common.h>
#include <basic/capsule.h>
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

namespace detail {
    const std::string default_description = "<EMPTY>";
}

class Robot {
public:
    Robot(const dWorldID &world, const dSpaceID &space)
    : world(world)
    , space(space)
    , bodies(world, space, constants::max_bodies)
    , joints(constants::max_joints)
    , accels(constants::max_accels)
    , attachments(world, space, constants::max_bodies)
    , description(detail::default_description)
    , cam_center_obj(0)
    , cam_setup(Vector3(0.3,-0.3,0.3), 130.,-18.,0.)
    , model_id()
    { }
    const dWorldID&  world;
    const dSpaceID&  space;

    SolidVector bodies; // body parts
    JointVector joints; // joints
    AccelVector accels; // acceleration sensors

    SolidVector attachments; // additional body parts

    std::string description; // individual robot's description

    std::size_t number_of_joints() const { return joints.get_size(); }
    std::size_t number_of_bodies() const { return bodies.size(); }
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
    void create_segment( const std::string name
                       , const Vector3 pos
                       , const Capsule cap
                       , const double mass
                       , const double density
                       , const Color4 color
                       , const bool collision
                       , const double friction = dInfinity);

    void attach_accel_sensor(std::string name_body, bool keep_original_color = false);

    void connect_joint (
                    std::string const& bodyname1, std::string const& bodyname2,
                    const double relx, const double rely, const double relz,
                    const char axis,
                    const double jointstopLo_deg, const double jointstopHi_deg, const double jointposDefault_deg,
                    const JointType Type,
                    const std::string Name,
                    const std::string SymName = "",
                    const double torque_factor = 5.0,
                    ActuatorParameters const& conf = ActuatorParameters()
                );

    void connect_joint (
                    std::string const& bodyname1, std::string const& bodyname2,
                    const Vector3 rel_pos,
                    const char axis,
                    const double jointstopLo_deg, const double jointstopHi_deg, const double jointposDefault_deg,
                    const JointType Type,
                    const std::string Name,
                    const std::string SymName = "",
                    const double torque_factor = 5.0,
                    ActuatorParameters const& conf = ActuatorParameters()
                );

    void connect_fixed(std::string const& bodyname1, std::string const& bodyname2);

    void attach_box( const std::string bodyname
                   , const Vector3 pos
                   , const Vector3 len
                   , const double mass
                   , const double density
                   , const Color4 color
                   , const bool collision
                   , const double friction = dInfinity) {

        std::string attach_name = bodyname + std::to_string(attachments.size());
        attachments.create_box(attach_name, pos, len, mass, density, color, collision, friction);

        unsigned bID = bodies     .get_body_id_by_name(bodyname);
        unsigned aID = attachments.get_body_id_by_name(attach_name);

        dJointID fixed = dJointCreateFixed(world, 0);
        dJointAttach(fixed, bodies[bID].body, attachments[aID].body);
        dJointSetFixed(fixed);
    }

    void attach_segment( const std::string bodyname
                       , const Vector3 pos
                       , const Capsule cap
                       , const double mass
                       , const double density
                       , const Color4 color
                       , const bool collision
                       , const double friction = dInfinity) {

        std::string attach_name = bodyname + std::to_string(attachments.size());
        attachments.create_capsule(attach_name, pos, cap, mass, density, color, collision, friction);

        unsigned bID = bodies     .get_body_id_by_name(bodyname);
        unsigned aID = attachments.get_body_id_by_name(attach_name);

        dJointID fixed = dJointCreateFixed(world, 0);
        dJointAttach(fixed, bodies[bID].body, attachments[aID].body);
    }

    void print_statistics(void) const;

    void set_camera_center_on(std::string const& name_body);
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
        for (std::size_t i = 0; i < attachments.size(); ++i)
            attachments[i].draw(conf.show_aabb);

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
        attachments.destroy();
        description = detail::default_description;
    }

private:
    unsigned int cam_center_obj;
    Camera_Setup cam_setup;

    ModelID model_id;
};


#endif // ROBOT_H_INCLUDED
