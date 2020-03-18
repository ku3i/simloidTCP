#ifndef BODIES_H_INCLUDED
#define BODIES_H_INCLUDED

#include <vector>
#include <string>
#include <cassert>
#include <draw/drawstuff.h>
#include <basic/vector3.h>
#include <basic/capsule.h>
#include <basic/color.h>
#include <basic/common.h>
#include <basic/draw.h>

dMass add(dMass const& m0, dMass const& m1);

/* general class for solid bodies */
class Solid {

    struct Geometry_t{
        Geometry_t(dGeomID const& id, Color4 const& color, double friction, bool collision)
        : id(id), color(color), friction(friction), collision(collision) {}

        dGeomID id;
        Color4  color;
        double friction; // 0..Inf
        bool collision;
    };

public:
    unsigned int id;
    std::string  name;
    dBodyID      body;
    std::vector<Geometry_t> geometries; // geometries representing this body for collision
    Vector3      force_to_draw;
    dJointID     fixed_joint;

private:
    Solid(dWorldID const& world, unsigned body_id, std::string const& solid_name, Vector3 const& pos)
    : id(body_id)
    , name(solid_name)
    , force_to_draw(.0)
    , fixed_joint(nullptr)
    {
        if (fabs(pos.x) > constants::max_position || fabs(pos.y) > constants::max_position || fabs(pos.z) > constants::max_position)
            dsError("Body is far too distant.\n");

        body = dBodyCreate(world);
        dBodySetPosition(body, pos.x, pos.y, pos.z);

        /* name */
        if (name == "")
            name = "solid_" + std::to_string(body_id);

        //dsPrint("Creating body %02u '%s'.\n", id, name.c_str());
    }


public:

    /* constructor for boxes */
    Solid( dWorldID const& world
         , dSpaceID const& space
         , unsigned body_id
         , std::string const& name
         , Vector3 const& pos
         , double friction
         , Color4 const& color
         , Vector3 const& len
         , double mass
         , double density
         , bool collision)
    : Solid(world, body_id, name, pos)
    {
        /* set mass for a box */
        dMass m;
        dMassSetZero(&m);
        if (mass > 0) dMassSetBoxTotal(&m, mass, len.x, len.y, len.z); // set mass if defined
        else dMassSetBox(&m, density, len.x, len.y, len.z);            // otherwise set density
        dBodySetMass(body, &m);

        /* geometry */
        geometries.emplace_back(dCreateBox(space, len.x, len.y, len.z), color, friction, collision);
        auto &g = geometries.back();
        dGeomSetBody(g.id, body);
        dGeomSetData (g.id, static_cast<void*>(&g.friction)); // save friction for near_callback
        if (!g.collision) dGeomDisable(g.id);
    }

    /* constructor for capsule segments */
    Solid( dWorldID const& world
         , dSpaceID const& space
         , unsigned body_id
         , std::string const& name
         , Vector3 const& pos
         , double friction
         , Color4 const& color
         , Capsule cap
         , double mass
         , double density
         , bool collision)
    : Solid(world, body_id, name, pos)
    {
        assert(cap.len >= 0);

        if (1 == cap.dir) { //TODO: make a method from that, to be able to rotate other objects as well
            dMatrix3 R;
            dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, constants::h_pi);
            dBodySetRotation(body, R);
        }
        else if (2 == cap.dir) {
            dMatrix3 R;
            dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, constants::h_pi);
            dBodySetRotation(body, R);
        }
        else assert(cap.dir == 3); // Direction must be 1=x, 2=y, 3=z!

        /* set mass and geometry for a capsule or sphere */
        dMass m;
        dMassSetZero(&m);

        if (cap.len == 0.)
        {
            if (mass > 0) dMassSetSphereTotal(&m, mass, cap.rad); // if mass > 0, set mass
            else dMassSetSphere(&m, density, cap.rad);            // otherwise set density
            geometries.emplace_back(dCreateSphere(space, cap.rad), color, friction, collision);
        } else {
            if (mass > 0) dMassSetCapsuleTotal(&m, mass, cap.dir, cap.rad, cap.len); // if mass > 0, set mass
            else dMassSetCapsule(&m, density, cap.dir, cap.rad, cap.len);            // otherwise set density
            geometries.emplace_back(dCreateCCylinder(space, cap.rad, cap.len), color, friction, collision);
        }

        dBodySetMass(body, &m);
        auto &g = geometries.back();
        dGeomSetBody(g.id, body);
        dGeomSetData (g.id, static_cast<void*>(&g.friction)); // save friction for near_callback
        if (!g.collision) dGeomDisable(g.id);
    }

    ~Solid()
    {
        if (fixed_joint != nullptr)
            dJointDestroy(fixed_joint);
        for (auto &g: geometries) dGeomDestroy(g.id);
        dBodyDestroy(body);
    }


    /* adding boxes to existing solids */
    void add_box( dSpaceID const& space
                , Vector3 const& rel
                , Vector3 const& len
                , double mass
                , double density
                , Color4 const& color
                , bool collision
                , double friction = dInfinity
                )
    {
        dMass m_body, m_add;
        dBodyGetMass (body, &m_body);

        /* set mass for a box */
        if (mass > 0) dMassSetBoxTotal(&m_add, mass, len.x, len.y, len.z); // set mass if defined
        else dMassSetBox(&m_add, density, len.x, len.y, len.z);            // otherwise set density

        ///TODO consider rotation
        //dMatrix3 drot;
		//dRFromAxisAndAngle (drot, 0,0,0,0);
		//dMassRotate (&m_add, drot); // rotate mass

        /* translate mass to relative position */
		dMassTranslate (&m_add, rel.x, rel.y, rel.z);

		/* add to the total mass */
		dMassAdd (&m_body, &m_add);

        dMassTranslate(&m_body, -m_body.c[0], -m_body.c[1], -m_body.c[2]);
	    dBodySetMass(body, &m_body);

        /* add geometry */
        geometries.emplace_back(dCreateBox(space, len.x, len.y, len.z), color, friction, collision);

        auto &g = geometries.back();
        dGeomSetBody(g.id, body);
        dGeomSetData (g.id, static_cast<void*>(&g.friction)); // save friction for near_callback
        if (!g.collision) dGeomDisable(g.id);

        dGeomSetOffsetPosition ( g.id
                               , rel.x - m_body.c[0]
			                   , rel.y - m_body.c[1]
			                   , rel.z - m_body.c[2] );

		//TODO: dGeomSetOffsetRotation(g, drot);

    }

    /* adding capsule-like segments */
    void add_segment( dSpaceID const& space
                    , Vector3 const& rel
                    , Capsule cap
                    , double mass
                    , double density
                    , Color4 const& color
                    , bool collision
                    , double friction = dInfinity )
    {
        assert(cap.len >= 0);

        /* set mass and geometry for a capsule or sphere */
        dMass m_body, m_add;
        dBodyGetMass(body, &m_body);

        if (cap.len == 0.) {
            if (mass > 0) dMassSetSphereTotal(&m_add, mass, cap.rad); // if mass > 0, set mass
            else dMassSetSphere(&m_add, density, cap.rad);            // otherwise set density
            geometries.emplace_back(dCreateSphere(space, cap.rad), color, friction, collision);
        } else {
            if (mass > 0) dMassSetCapsuleTotal(&m_add, mass, cap.dir, cap.rad, cap.len); // if mass > 0, set mass
            else dMassSetCapsule(&m_add, density, cap.dir, cap.rad, cap.len);            // otherwise set density
            geometries.emplace_back(dCreateCCylinder(space, cap.rad, cap.len), color, friction, collision);
        }

        ///TODO consider rotation
        //dMatrix3 drot;
		//dRFromAxisAndAngle (drot, 0,0,0,0);
		//dMassRotate (&m_add, drot); // rotate mass

        /* translate mass to relative position */
		dMassTranslate (&m_add, rel.x, rel.y, rel.z);

		/* add to the total mass */
		dMassAdd (&m_body, &m_add);

        dMassTranslate(&m_body, -m_body.c[0], -m_body.c[1], -m_body.c[2]);
	    dBodySetMass(body, &m_body);

        auto &g = geometries.back();
	    dGeomSetBody(g.id, body);
	    dGeomSetData (g.id, static_cast<void*>(&g.friction)); // save friction for near_callback
        if (!g.collision) dGeomDisable(g.id);

        if (1 == cap.dir) {
            dMatrix3 R;
            dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, constants::h_pi);
            dGeomSetOffsetRotation(g.id, R);
        }
        else if (2 == cap.dir) {
            dMatrix3 R;
            dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, constants::h_pi);
            dGeomSetOffsetRotation(g.id, R);
        }
        else assert(cap.dir == 3); // Direction must be 1=x, 2=y, 3=z!

        dGeomSetOffsetPosition ( g.id
                               , rel.x - m_body.c[0]
			                   , rel.y - m_body.c[1]
			                   , rel.z - m_body.c[2] );
    }


    void draw(bool bounding_box)
    {
        for (auto const& g: geometries) {
            if (g.color == colors::invisible) continue;
            dsSetColorAlpha(g.color.r, g.color.g, g.color.b, g.color.a);
            drawGeom(g.id, 0, 0, bounding_box);
        }

        if (not force_to_draw.is_zero()) {
            const Vector3& pos0 = dBodyGetPosition(body);
            const Vector3& pos1 = pos0 + force_to_draw;

            dsSetColorAlpha(colors::cyan.r, colors::cyan.g, colors::cyan.b, colors::cyan.a);
            draw_line(pos0, pos1);
            force_to_draw = 0;
        }
    }

    Vector3 get_position(void) const { return Vector3(dBodyGetPosition (body)); }
    Vector3 get_velocity(void) const { return Vector3(dBodyGetLinearVel(body)); }

    void set_impulse(const Vector3& force) { dBodyAddForce(body, force.x, force.y, force.z); force_to_draw = clip(0.1*force, 1.0); }

    void toggle_fixed(const dWorldID& world)
    {
        if (fixed_joint == nullptr) {
            dsPrint("Fixating solid: '%s'\n", name.c_str());
            fixed_joint = dJointCreateFixed(world, 0);
            dJointAttach(fixed_joint, body, 0);
            dJointSetFixed(fixed_joint);
        } else {
            dsPrint("Releasing solid: '%s'\n", name.c_str());
            dJointDestroy(fixed_joint);
            fixed_joint = nullptr;
        }
    }
};

class SolidVector
{
public:
    SolidVector( const dWorldID&   world
               , const dSpaceID&   space
               , const std::size_t max_number_of_bodies)
    : world(world)
    , space(space)
    , bodies()
    , max_number_of_bodies(max_number_of_bodies)
    {
        dsPrint("Creating body vector...");
        bodies.reserve(max_number_of_bodies);
        dsPrint("done.\n");
    }

    ~SolidVector() { dsPrint("Destroying bodies.\n"); }

    unsigned int create_box(const std::string& name,
                            const Vector3& pos, const Vector3& len,
                            const double mass, const double density,
                            const Color4& color,
                            const bool collision, const double friction)
    {
        unsigned int body_id = bodies.size();
        if (body_id < max_number_of_bodies)
        {
            if (get_body_id_by_name(name) < body_id) {
                dsError("Name '%s' already in use.", name.c_str());
            }
            bodies.emplace_back(world, space, body_id, name, pos, friction, color, len, mass, density, collision);
        } else {
            dsError("Exceeded maximum number of bodies %u.", max_number_of_bodies);
        }
        return body_id;
    }

    unsigned int create_capsule(const std::string& name,
                                const Vector3& pos,
                                const Capsule cap,
                                const double mass, const double density,
                                const Color4& color,
                                const bool collision, const double friction)
    {
        unsigned int body_id = bodies.size();
        if (body_id < max_number_of_bodies)
        {
            if (get_body_id_by_name(name) < body_id) {
                dsError("Name '%s' already in use.", name.c_str());
            }
            bodies.emplace_back(world, space, body_id, name, pos, friction, color, cap, mass, density, collision);
        } else {
            dsError("Exceeded maximum number of bodies %u.", max_number_of_bodies);
        }
        return body_id;
    }

    unsigned int get_body_id_by_name(std::string const& name) const
    {
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (name == bodies[i].name)
                return i;
        return bodies.size();
    }

    Solid& get_body_by_name(std::string const& name) {
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (name == bodies[i].name)
                return bodies[i];
        assert(false);
    }

          Solid& operator[](std::size_t idx)       { return bodies.at(idx); /**TODO use normal index if everything works out*/}
    const Solid& operator[](std::size_t idx) const { return bodies.at(idx); }

    std::size_t size() const { return bodies.size(); }

    dMass get_total_mass(void) const {
        dMass total_mass;
        dMassSetZero(&total_mass);
        if (bodies.size() == 0) return total_mass;
        for (auto const& b : bodies) {
            dMass add_mass;
            dBodyGetMass(b.body, &add_mass);
            const dReal* pos= dBodyGetPosition(b.body);
            dMassTranslate(&add_mass, pos[0], pos[1], pos[2]);
            dMassAdd(&total_mass, &add_mass);
        }
        return total_mass;
    }

    void destroy(void) {
        dsPrint("Destroying bodies (for recreation).\n");
        bodies.clear();
    }

private:
    const dWorldID&  world;
    const dSpaceID&  space;
    std::vector<Solid> bodies;
    const std::size_t  max_number_of_bodies;
};

#endif // BODIES_H_INCLUDED
