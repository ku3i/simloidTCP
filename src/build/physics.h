#ifndef PHYSICS_H_INCLUDED
#define PHYSICS_H_INCLUDED

#include <ode/ode.h>

#include <draw/drawstuff.h>
#include <basic/configuration.h>
#include <basic/constants.h>

extern Configuration global_conf;

void near_callback(void *data, dGeomID o1, dGeomID o2);

class physics {
public:
    physics()
    {
        dsPrint("Creating the world.\n");
        dInitODE();
        world = dWorldCreate();
        space = dHashSpaceCreate(0);
        ground = dCreatePlane (space, 0, 0, 1, 0); // plane equation is 0x + 0y + 1z = 0
        contactgroup = dJointGroupCreate(0);

        /* init Gravity on/off */
        set_gravity(global_conf.initial_gravity);

        dWorldSetCFM (world, constants::world_CFM);
        dWorldSetERP (world, constants::world_ERP);

        dsPrint("The world has been created.\n");
    }

    ~physics() {
        dsPrint("Destroying ground, world and space.\n");
        dJointGroupDestroy(contactgroup);
        dGeomDestroy(ground);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        dsPrint("All has been destroyed. Closing ODE.\n");
        dCloseODE();
        dsPrint("All done, Simloid says goodbye______\n");
    }

    void set_gravity(bool enable) const {
        if (enable) {
            dWorldSetGravity (world, .0, .0, -constants::gravity);
            dsPrint("Gravity on.\n");
        } else {
            dWorldSetGravity (world, .0, .0, .0);
            dsPrint("Gravity off.\n");
        }
    }

    dWorldID       world;
    dSpaceID       space;
    dGeomID        ground;
    dJointGroupID  contactgroup;
};

#endif // PHYSICS_H_INCLUDED
