#include <basic/snapshot.h>

void recordSnapshot(const Robot& robot, const Obstacle& obstacles, Snapshot *s)
{
    dsPrint("Recording snapshot.\n");
    /* robot */
    for (unsigned int i = 0; i < robot.number_of_bodies(); ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            s->bodystate[i].position  [j] = (dBodyGetPosition  (robot.bodies[i].body))[j]; // Position
            s->bodystate[i].linearVel [j] = (dBodyGetLinearVel (robot.bodies[i].body))[j]; // Linear Velocity
            s->bodystate[i].angularVel[j] = (dBodyGetAngularVel(robot.bodies[i].body))[j]; // Angular Velocity
        }

        for (unsigned int j = 0; j < 4; ++j)
            s->bodystate[i].quaternion[j] = (dBodyGetQuaternion(robot.bodies[i].body))[j]; // Quaternion

        const dReal *tmp;
        tmp = dBodyGetRotation(robot.bodies[i].body);
        for (unsigned int j = 0; j < 12; ++j)
            s->bodystate[i].rotation[j] = tmp[j]; // Rotation
    }

    /* obstacles */
    for (unsigned int i = 0; i < obstacles.number_of_objects(); ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            s->obststate[i].position  [j] = (dBodyGetPosition  (obstacles.objects[i].body))[j]; // Position
            s->obststate[i].linearVel [j] = (dBodyGetLinearVel (obstacles.objects[i].body))[j]; // Linear Velocity
            s->obststate[i].angularVel[j] = (dBodyGetAngularVel(obstacles.objects[i].body))[j]; // Angular Velocity
        }

        for (unsigned int j = 0; j < 4; ++j)
            s->obststate[i].quaternion[j] = (dBodyGetQuaternion(obstacles.objects[i].body))[j]; // Quaternion

        const dReal *tmp;
        tmp = dBodyGetRotation(obstacles.objects[i].body);
        for (unsigned int j = 0; j < 12; ++j)
            s->obststate[i].rotation[j] = tmp[j]; // Rotation
    }
}

void playSnapshot(const Robot& robot, const Obstacle& obstacles, const Snapshot *s)
{
    /* robot */
    for (unsigned int i = 0; i < robot.number_of_bodies(); ++i)
    {
        //Position
        dBodySetPosition  ( robot.bodies[i].body
                          , s->bodystate[i].position[0]
                          , s->bodystate[i].position[1]
                          , s->bodystate[i].position[2] );

        //LinearVel
        dBodySetLinearVel ( robot.bodies[i].body
                          , s->bodystate[i].linearVel[0]
                          , s->bodystate[i].linearVel[1]
                          , s->bodystate[i].linearVel[2] );

        //AngularVel
        dBodySetAngularVel( robot.bodies[i].body
                          , s->bodystate[i].angularVel[0]
                          , s->bodystate[i].angularVel[1]
                          , s->bodystate[i].angularVel[2] );

        //Quaternion
        dBodySetQuaternion(robot.bodies[i].body, s->bodystate[i].quaternion);

        //Rotation
        dBodySetRotation(robot.bodies[i].body, s->bodystate[i].rotation);
    }

    /* obstacles */
    for (unsigned int i = 0; i < obstacles.number_of_objects(); ++i)
    {
        //Position
        dBodySetPosition  ( obstacles.objects[i].body
                          , s->obststate[i].position[0]
                          , s->obststate[i].position[1]
                          , s->obststate[i].position[2] );

        //LinearVel
        dBodySetLinearVel ( obstacles.objects[i].body
                          , s->obststate[i].linearVel[0]
                          , s->obststate[i].linearVel[1]
                          , s->obststate[i].linearVel[2] );

        //AngularVel
        dBodySetAngularVel( obstacles.objects[i].body
                          , s->obststate[i].angularVel[0]
                          , s->obststate[i].angularVel[1]
                          , s->obststate[i].angularVel[2] );

        dBodySetQuaternion(obstacles.objects[i].body, s->obststate[i].quaternion); // Quaternion
        dBodySetRotation  (obstacles.objects[i].body, s->obststate[i].rotation  ); // Rotation
    }
}
