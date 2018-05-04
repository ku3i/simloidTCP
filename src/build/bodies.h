#ifndef BODIES_H_INCLUDED
#define BODIES_H_INCLUDED

#include <vector>
#include <string>
#include <draw/drawstuff.h>
#include <basic/vector3.h>
#include <basic/color.h>
#include <basic/common.h>
#include <basic/draw.h>

/* general class for solid bodies */
class Solid {
public:
    unsigned int id;
    std::string  name;
    dBodyID      body;     // physics: the body
    dGeomID      geometry; // geometry representing this body for collision
    dReal        friction; // 0..Inf
    Color4       color;
    Vector3      force_to_draw;
    dJointID     fixed_joint;

private:
    Solid(const dWorldID &world, unsigned int body_id, const std::string& solid_name, const Vector3& pos, const double frict, const Color4& color)
    : id(body_id)
    , name(solid_name)
    , friction((frict > 0)? frict : .0) // avoid negative values
    , color(color)
    , force_to_draw(.0)
    {
        if (fabs(pos.x) > constants::max_position || fabs(pos.y) > constants::max_position || fabs(pos.z) > constants::max_position)
            dsError("Body is far too distant.\n");

        body = dBodyCreate(world);
        dBodySetPosition(body, pos.x, pos.y, pos.z);

        /* name */
        if (name == "")
            name = "solid_" + std::to_string(body_id);

        dsPrint("Creating body %02u '%s'.\n", id, name.c_str());
    }

public:

    /* constructor for boxes */
    Solid( const dWorldID& world
         , const dSpaceID& space
         , const unsigned int body_id
         , const std::string& name
         , const Vector3& pos
         , const double frict
         , const Color4& color
         , const Vector3& len
         , const double mass
         , const double density
         , const bool collision)
    : Solid(world, body_id, name, pos, frict, color)
    {
        /* set mass for a box */
        dMass m;
        dMassSetZero(&m);
        if (mass > 0) dMassSetBoxTotal(&m, mass, len.x, len.y, len.z); // set mass if defined
        else dMassSetBox(&m, density, len.x, len.y, len.z);            // otherwise set density
        dBodySetMass(body, &m);

        /* geometry */
        geometry = dCreateBox(space, len.x, len.y, len.z);
        dGeomSetBody(geometry, body);
        dGeomSetData (geometry, static_cast<void*>(&friction)); // save friction for near_callback
        if (!collision) dGeomDisable(geometry);
    }
    /* constructor for capsule segments */
    Solid( const dWorldID &world
         , const dSpaceID &space
         , const unsigned int body_id
         , const std::string& name
         , const Vector3& pos
         , const double frict
         , const Color4& color
         , const unsigned int direction
         , const double length
         , const double radius
         , const double mass
         , const double density
         , const bool collision)
    : Solid(world, body_id, name, pos, frict, color)
    {
        /* check direction */
        if (direction < 1 || direction > 3) {
            dsError("'Direction' should be 1=x, 2=y, 3=z.");
        }
        if (1 == direction) { //TODO: make a method from that, to be able to rotate other objects as well
            dMatrix3 R;
            dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, constants::h_pi);
            dBodySetRotation(body, R);
        }
        else if (2 == direction) {
            dMatrix3 R;
            dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, constants::h_pi);
            dBodySetRotation(body, R);
        }
        //else do nothing

        /* set mass for a capsule */
        dMass m;
        dMassSetZero(&m);
        if (mass > 0) dMassSetCapsuleTotal(&m, mass, direction, radius, length); // if mass > 0, set mass
        else dMassSetCapsule(&m, density, direction, radius, length);            // otherwise set density
        dBodySetMass(body, &m);

        /* geometry */
        geometry = dCreateCCylinder(space, radius, length);
        dGeomSetBody(geometry, body);
        dGeomSetData(geometry, static_cast<void*>(&friction)); // save friction for near_callback
        if (!collision) dGeomDisable(geometry);
    }

    ~Solid()
    {
        dsPrint("Destroying body %02u '%s'.\n", id, name.c_str());
        dGeomDestroy(geometry);
        dBodyDestroy(body);
    }

    void draw(bool bounding_box)
    {
        if (color == colors::invisible) return;
        dsSetColorAlpha(color.r, color.g, color.b, color.a);
        drawGeom(geometry, 0, 0, bounding_box);


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

    void fixed(const dWorldID& world)
    {
        fixed_joint = dJointCreateFixed(world, 0);
        dJointAttach(fixed_joint, body, 0);
        dJointSetFixed(fixed_joint);
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
                                const unsigned int direction, const double length, const double radius,
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
            bodies.emplace_back(world, space, body_id, name, pos, friction, color, direction, length, radius, mass, density, collision);
        } else {
            dsError("Exceeded maximum number of bodies %u.", max_number_of_bodies);
        }
        return body_id;
    }

    unsigned int get_body_id_by_name(const std::string& name) const
    {
        for (unsigned int i = 0; i < bodies.size(); ++i)
            if (name == bodies[i].name)
                return i;
        return bodies.size();
    }

          Solid& operator[](std::size_t idx)       { return bodies[idx]; }
    const Solid& operator[](std::size_t idx) const { return bodies[idx]; }

    std::size_t get_size() const { return bodies.size(); }

    dMass get_total_mass(void) const {
        dMass total_mass;
        dMassSetZero(&total_mass);
        for (std::size_t i = 0; i < get_size(); ++i) {
            dMass add_mass;
            dBodyGetMass(bodies[i].body, &add_mass);
            const dReal* pos= dBodyGetPosition(bodies[i].body);
            dMassTranslate(&add_mass, pos[0], pos[1], pos[2]);
            dMassAdd(&total_mass, &add_mass);
        }
        return total_mass;
    }

private:
    const dWorldID&  world;
    const dSpaceID&  space;
    std::vector<Solid> bodies;
    const std::size_t  max_number_of_bodies;
};



#endif // BODIES_H_INCLUDED

/*

//Example code for composite objects:
#define GPB 3

dGeomID g2[GPB];		// encapsulated geometries
dReal dpos[GPB][3];	    // delta-positions for encapsulated geometries

// start accumulating masses for the encapsulated geometries
dMass m2;
dMassSetZero (&m);


for (k = 0; k < GPB; k++) {
    object.geom[k] = dCreateGeomTransform (space);
	dGeomTransformSetCleanup (object.geom[k], 1);

	g2[k] = dCreateBox (0,sides[0],sides[1],sides[2]);
	dMassSetBox (&m2,DENSITY,sides[0],sides[1],sides[2]);

	dGeomTransformSetGeom (object.geom[k], g2[k]);

	// set the transformation (adjust the mass too)
	dGeomSetPosition (g2[k], dpos[k][0], dpos[k][1], dpos[k][2]);
//	dMatrix3 Rtx;
//	dRFromAxisAndAngle (Rtx,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
//			    dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
//	dGeomSetRotation (g2[k],Rtx);
//	dMassRotate (&m2,Rtx);

	// Translation *after* rotation
	dMassTranslate (&m2, dpos[k][0], dpos[k][1], dpos[k][2]);

	// add to the total mass
	dMassAdd (&m,&m2);
}

for (k=0; k < GPB; k++)
    if (obj[i].geom[k])
        dGeomSetBody (object.geom[k], object.body);


    dBodySetMass (object.body,&m);

*/

//HOWTO composite objects
//
//This article is known to contain outdated information. If you know of correct information, please edit this article! Thank you.
//
//Copy-dump from old wiki! Is it still up-to-date (notably the GeomTransform parts) ?
//Contents
//Introduction
//
//ODE allows you to have composite objects. These are more complex objects build from several others. For instance a table could be built by combining four leg boxes with a table-top box.
//
//The ODE documentation tells you which functions to call, and there's a "box stack" demo which includes composite objects. It can be a little tricky to get to grips with, so here's a page on the subject.
//Theory
//
//A body can contain multiple geoms. However to build a useful composite object you're likely going to need to separate the geoms in space. To do this you need to wrap the collision geoms in "GeomTransforms". A GeomTransform is a geom which has no collision, it just wraps another geom and allows it to be translated/rotated. So the hierarchy for a simple table would look like this:
//
//dBody              // Table composite object
//    dGeomTransform // Transform for offsetting table-top
//        dGeomBox   // Table-top collision geom
//    dGeomTransform // Transform for offsetting leg
//        dGeomBox   // Leg 1 collision geom
//    dGeomTransform // Transform for offsetting leg
//        dGeomBox   // Leg 2 collision geom
//    dGeomTransform // Transform for offsetting leg
//        dGeomBox   // Leg 3 collision geom
//    dGeomTransform // Transform for offsetting leg
//        dGeomBox   // Leg 4 collision geom
//
//Building a Composite Object
//
//The following code has been copied from the "box stack" sample, cleaned, and commented. The code has also been simplified in that it only uses boxes for the sub objects.
//
// const int NUM_BOXES = 3;
//
// dGeomID odeGeomTransformList[NUM_BOXES]; /* Geometry transforms which contain single sub objects */
// dGeomID tempOdeBoxGeomList[NUM_BOXES]; /* Temp array for holding onto the boxes in the transforms.
//                                         * This doesn't need to be kept, as dGeomTransformSetCleanup
//                                         * will take care of it. */
// dReal boxOffset[NUM_BOXES][3]; /* delta-positions for encapsulated geometries */
//
// /* Start accumulating masses for the encapsulated geometries */
// dMass compositeMass, componentMass;
// dMassSetZero (&compositeMass);
//
// /* Pick random positions geoms will be placed at */
// for (j=0; j<NUM_BOXES; j++)
// {
//     /* Loop through XYZ */
//     for (k=0; k<3; k++)
//     {
//         boxOffset[j][k] = dRandReal()*0.3-0.15;
//     }
// }
//
// for (k=0; k<NUM_BOXES; k++)
// {
//     odeGeomTransformList[k] = dCreateGeomTransform (space); /* Create geom transform that sub
//                                                              * objects are added to */
//     dGeomTransformSetCleanup (odeGeomTransformList[k],1);/* If '1' deleting odeGeomTransformList[k]
//                                                           * will delete the geom added to it */
//
//     tempOdeBoxGeomList[k] = dCreateBox (0, sides[0], /* Create box sub-object */
//                                         sides[1],
//                                         sides[2]);
//     dMassSetBox (&componentMass, /* Create normal mass for a box */
//                  DENSITY,
//                  sides[0],
//                  sides[1],
//                  sides[2]);
//
//     dGeomTransformSetGeom (odeGeomTransformList[k], /* Attach box to the geom transform */
//                            tempOdeBoxGeomList[k]);
//
//     /* Move and rotate the box within the compound object, including the mass */
//     dGeomSetPosition (tempOdeBoxGeomList[k],
//                       boxOffset[k][0],
//                       boxOffset[k][1],
//                       boxOffset[k][2]);
//     dMassTranslate (&componentMass,
//                     boxOffset[k][0],
//                     boxOffset[k][1],
//                     boxOffset[k][2]);
//     dMatrix3 rotation;
//     dRFromAxisAndAngle (rotation,
//                         dRandReal()*2.0-1.0,
//                         dRandReal()*2.0-1.0,
//                         dRandReal()*2.0-1.0,
//                         dRandReal()*10.0-5.0);
//     dGeomSetRotation (tempOdeBoxGeomList[k], rotation);
//     dMassRotate (&componentMass, rotation);
//
//     /* Add sub object mass to the total mass */
//     dMassAdd (&compositeMass,&componentMass);
// }
//
// // We've finished adding object. The center of mass is not going to be centered at the moment.
// // Move all encapsulated objects so that the center of mass is (0,0,0)
// for (k=0; k<NUM_BOXES; k++)
// {
//     dGeomSetPosition (tempOdeBoxGeomList[k],
//                       boxOffset[k][0]-compositeMass.c[0],
//                       boxOffset[k][1]-compositeMass.c[1],
//                       boxOffset[k][2]-compositeMass.c[2]);
// }
// dMassTranslate (&compositeMass,
//                 -compositeMass.c[0],
//                 -compositeMass.c[1],
//                 -compositeMass.c[2]);
//
// /* Tell all the geom transforms that they belong to the parent body */
// for (k=0; k < NUM_BOXES; k++)
// {
//     if (odeGeomTransformList?[k])
//         dGeomSetBody (odeGeomTransformList[k], parentBody );
// }
//
// /* Set the parent body mass to be the one we've calculated. */
// dBodySetMass (parentBody, &compositeMass );
//
//Getting the position of a composite object component
//
//If you get the position of a composite object component, ODE will return the relative/local position within the body. The code that follows is for converting this to a real world position. Again the code originates from the 'box stack' sample. Apologies that it's not in straight C calls, but it's quick to change to whatever you're using.
//
// // Get the geomID of the component from the geom transform
// dGeomID geomID = dGeomTransformGetGeom( geomTransformID? );
//
// // Get world position/orientation of parent body
// const Real* pBodyPos = parentBody->getPosition();
// const Real* pBodyRotMat = parentBody->getRotation();
//
// // Get local position/orientation of component within geom transform
// const Real* pLocalPos = dGeomGetPosition (geomID);
// const Real* pLocalRotMat = dGeomGetRotation (geomID);
//
// // Calculate world space position of component by using the body position/orientation
// dVector3 worldPos;
// dMatrix3 worldRotMat;
// dMULTIPLY0_331( worldPos, pBodyRotMat, pLocalPos );
// worldPos[0] += pBodyPos[0];
// worldPos[1] += pBodyPos[1];
// worldPos[2] += pBodyPos[2];
// dMULTIPLY0_333( worldRotMat, pBodyRotMat, pLocalRotMat );
//
// // And if you want a quaternion instead of a matrix...
// dQuaternion worldQuat;
// dQfromR(worldQuat, worldRotMat);
//
