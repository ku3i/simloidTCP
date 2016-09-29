#include <build/physics.h>

/* this is called by dSpaceCollide when two objects in space are potentially colliding */
void near_callback(void *data, dGeomID o1, dGeomID o2)
{
    physics *universe = static_cast<physics *>(data);

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    dReal mu1 = 1;
    dReal mu2 = 1;

    if (!dGeomIsSpace(o1)) {
        dReal *friction1 = static_cast<dReal*>(dGeomGetData(o1));
        if (friction1 != NULL)
            mu1 = *friction1;
    }
    if (!dGeomIsSpace(o2)) {
        dReal *friction2 = static_cast<dReal*>(dGeomGetData(o2));
        if (friction2 != NULL)
            mu2 = *friction2;
    }

    /* exit without doing anything if the two bodies are connected by a joint */
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

    dContact contact[constants::max_contacts];   // up to constants::max_contacts contacts per box-box
    for (unsigned int i = 0; i < constants::max_contacts; ++i)
    {
        contact[i].surface.mode = dContactSoftCFM | dContactSoftERP
        // | dContactApprox1
        | dContactSlip1 | dContactSlip2
        // | dContactBounce
        ;

        contact[i].surface.mu = std::max(mu1, mu2); // dInfinity = extreme sticky objects
        contact[i].surface.slip1 = 0.001;
        contact[i].surface.slip2 = 0.001;
        contact[i].surface.soft_cfm = global_conf.contact_soft_CFM;
        contact[i].surface.soft_erp = global_conf.contact_soft_ERP;
    }

    if (int numc = dCollide (o1, o2, constants::max_contacts, &contact[0].geom, sizeof(dContact)))
    {
        dMatrix3 RI;
        dRSetIdentity (RI);
        const dReal size[3] = {0.02, 0.02, 0.02};

        for (int i = 0; i < numc; ++i)
        {
            dJointID c = dJointCreateContact(universe->world, universe->contactgroup, contact + i);
            dJointAttach (c, b1, b2);
            if (!global_conf.disable_graphics && global_conf.show_contacts)
                dsDrawBox ((const double *) contact[i].geom.pos, (const double *) RI, (const double *) size);
        }
    }
}
