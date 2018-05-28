#ifndef SNAPSHOT_H_INCLUDED
#define SNAPSHOT_H_INCLUDED

#include <build/robot.h>
#include <build/obstacles.h>

/**
 * TODO re-factor this:
 * make it a class
 * add controller state
 */

struct BodyState {
    dReal       position[3];
    dMatrix3    rotation;
    dQuaternion quaternion;
    dReal       linearVel[3];
    dReal       angularVel[3];
};


struct Snapshot {
    ModelID model_id;
    struct BodyState bodystate[constants::max_bodies];
    struct BodyState obststate[constants::max_obstacles];
};

void recordSnapshot(const Robot& robot, const Obstacle& obstacles, Snapshot *s);
void playSnapshot  (const Robot& robot, const Obstacle& obstacles, const Snapshot *s);

#endif // SNAPSHOT_H_INCLUDED
