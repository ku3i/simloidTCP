#include <sensors/accelsensor.h>

const Vector3 AccelSensor::update(void)
{
    /* get current velocity */
    const Vector3 current_velocity(dBodyGetLinearVel(body_id));

    /* differentiate */
    acceleration = (current_velocity - last_velocity) / dt;
    last_velocity = current_velocity; // remember last velocity

    /* add gravity */
    acceleration -= gravity;

    /* get rotation of body */
    const dReal* r = dBodyGetRotation(body_id); // TODO use quaternion

    // Richtung 1, r_dir1  <- dir1 drehen
    const Vector3 r_dir1 (r[0]*dir[0][0] + r[1]*dir[0][1] +  r[2]*dir[0][2],
                          r[4]*dir[0][0] + r[5]*dir[0][1] +  r[6]*dir[0][2],
                          r[8]*dir[0][0] + r[9]*dir[0][1] + r[10]*dir[0][2]);

    // Richtung 2, r_dir2 <- dir2 drehen
    const Vector3 r_dir2 (r[0]*dir[1][0] + r[1]*dir[1][1] +  r[2]*dir[1][2],
                          r[4]*dir[1][0] + r[5]*dir[1][1] +  r[6]*dir[1][2],
                          r[8]*dir[1][0] + r[9]*dir[1][1] + r[10]*dir[1][2]);

    // Richtung 3, r_dir3 <- dir3 drehen
    const Vector3 r_dir3 (r[0]*dir[2][0] + r[1]*dir[2][1] +  r[2]*dir[2][2],
                          r[4]*dir[2][0] + r[5]*dir[2][1] +  r[6]*dir[2][2],
                          r[8]*dir[2][0] + r[9]*dir[2][1] + r[10]*dir[2][2]);

    /* project to sensor axis */
    Vector3 sensor(.0);
    sensor.x = acceleration * r_dir1;
    sensor.y = acceleration * r_dir2;
    sensor.z = acceleration * r_dir3;

    /* scale to range, clip to [-1,1) add 4 bit of noise
     * and lower to 16 bit resolution */
    sensor /= constants::gravity * constants::range_accelsensor;
    sensor.clip(1.0);
    sensor.low_resolution();

    return sensor;
}

void
AccelSensor::draw(void)
{
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal size[3] = {0.1, 0.1, 0.1};
    const dReal size2[3] = {0.101, 0.101, 0.101};

    const Vector3 body_position(dBodyGetPosition(body_id));
    const double position[3] = {body_position.x - 0.001*acceleration.x, body_position.y - 0.001*acceleration.y, body_position.z - 0.001*acceleration.z};

    dsSetColorAlpha (0.0, 1.0, 0, 1.0);
    dsDrawBox (dBodyGetPosition(body_id), dBodyGetRotation(body_id), (const double *) size);
    dsSetColorAlpha (1.0, 1.0, 0, .5);
    dsDrawBox (position, dBodyGetRotation(body_id), (const double *) size2);
}
