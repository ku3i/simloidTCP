#ifndef DRAW_H_INCLUDED
#define DRAW_H_INCLUDED

#include <ode/ode.h>
#include <draw/drawstuff.h>
#include <basic/constants.h>
#include <basic/vector3.h>

void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb);
void draw_line(const Vector3 _pos1, const Vector3 _pos2);


#endif // DRAW_H_INCLUDED
