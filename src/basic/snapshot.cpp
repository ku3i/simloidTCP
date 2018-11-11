#include <basic/snapshot.h>


template <typename ObjectList, typename StateList>
void record_objects(const ObjectList& objects, StateList& s) {
    for (unsigned int i = 0; i < objects.size(); ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            s[i].position  [j] = (dBodyGetPosition  (objects[i].body))[j]; // Position
            s[i].linearVel [j] = (dBodyGetLinearVel (objects[i].body))[j]; // Linear Velocity
            s[i].angularVel[j] = (dBodyGetAngularVel(objects[i].body))[j]; // Angular Velocity
        }

        for (unsigned int j = 0; j < 4; ++j)
            s[i].quaternion[j] = (dBodyGetQuaternion(objects[i].body))[j]; // Quaternion

        const dReal *tmp;
        tmp = dBodyGetRotation(objects[i].body);
        for (unsigned int j = 0; j < 12; ++j)
            s[i].rotation[j] = tmp[j]; // Rotation
    }
}

template <typename ObjectList, typename StateList>
void restore_objects(const ObjectList& objects, const StateList& s)
{
    for (unsigned int i = 0; i < objects.size(); ++i)
    {
        dBodySetPosition  ( objects[i].body
                          , s[i].position[0]
                          , s[i].position[1]
                          , s[i].position[2] );

        dBodySetLinearVel ( objects[i].body
                          , s[i].linearVel[0]
                          , s[i].linearVel[1]
                          , s[i].linearVel[2] );

        dBodySetAngularVel( objects[i].body
                          , s[i].angularVel[0]
                          , s[i].angularVel[1]
                          , s[i].angularVel[2] );

        dBodySetQuaternion(objects[i].body, s[i].quaternion);
        dBodySetRotation  (objects[i].body, s[i].rotation  );
    }
}

void recordSnapshot(const Robot& robot, const Obstacle& obstacles, Snapshot *s)
{
    s->model_id = robot.get_model_id();

    dsPrint("Recording snapshot.\n");

    record_objects(robot.bodies     , s->bodies     );
    record_objects(robot.attachments, s->attachments);
    record_objects(obstacles.objects, s->obstacles  );
}

void playSnapshot(const Robot& robot, const Obstacle& obstacles, const Snapshot *s)
{
    assert(robot.get_model_id() == s->model_id);

    restore_objects(robot.bodies     , s->bodies     );
    restore_objects(robot.attachments, s->attachments);
    restore_objects(obstacles.objects, s->obstacles  );
}
