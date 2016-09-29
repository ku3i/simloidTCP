#include "./draw.h"


void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
    if (!g) return;
    if (!pos) pos = dGeomGetPosition (g);
    if (!R) R = dGeomGetRotation (g);

    int type = dGeomGetClass (g);
    if (type == dBoxClass)
    {
        dVector3 sides;
        dGeomBoxGetLengths (g,sides);
        dsDrawBox ((const double *)pos,(const double *)R,(const double *)sides);
    }
    else if (type == dSphereClass)
    {
        dsDrawSphere ((const double *)pos,(const double *)R,dGeomSphereGetRadius (g));
    }
    else if (type == dCCylinderClass)
    {
        dReal radius,length;
        dGeomCCylinderGetParams (g,&radius,&length);
        dsDrawCappedCylinder ((const double *)pos,(const double *)R,length,radius);
    }
    else if (type == dGeomTransformClass)
    {
        dGeomID g2 = dGeomTransformGetGeom (g);
        const dReal *pos2 = dGeomGetPosition (g2);
        const dReal *R2 = dGeomGetRotation (g2);
        dVector3 actual_pos;
        dMatrix3 actual_R;
        dMULTIPLY0_331 (actual_pos,R,pos2);
        actual_pos[0] += pos[0];
        actual_pos[1] += pos[1];
        actual_pos[2] += pos[2];
        dMULTIPLY0_333 (actual_R,R,R2);
        drawGeom (g2,actual_pos,actual_R,0);
    }

    if (show_aabb)
    {
        // draw the bounding box for this geom
        dReal aabb[6];
        dGeomGetAABB(g,aabb);
        dVector3 bbpos;
        for (unsigned int i = 0; i < 3; ++i)
            bbpos[i] = 0.5 * (aabb[i*2] + aabb[i*2 + 1]);
        dVector3 bbsides;
        for (unsigned int i = 0; i < 3; ++i)
            bbsides[i] = aabb[i*2 + 1] - aabb[i*2];
        dMatrix3 RI;
        dRSetIdentity (RI);
        dsSetColorAlpha (1, 0, 0, 0.5);
        dsDrawBox((const double *)bbpos,(const double *)RI,(const double *)bbsides);
    }
}


void draw_line(const Vector3 _pos1, const Vector3 _pos2)
{
    double pos1[3], pos2[3];
    pos1[0] = _pos1.x;
    pos1[1] = _pos1.y;
    pos1[2] = _pos1.z;
    pos2[0] = _pos2.x;
    pos2[1] = _pos2.y;
    pos2[2] = _pos2.z;
    dsDrawLineD(pos1,pos2); // <- this method should be re-factored
}
