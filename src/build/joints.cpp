#include "./joints.h"

bool JointVector::add_symmetric(std::size_t joint_id, std::string sym_name)
{
    bool result = false;

    if (JointType::symmetric == joints[joint_id].type) {
        for (std::size_t j = 0; j < joints.size(); ++j) {
            if (joints[j].name == sym_name) {
                dsPrint("Joint %02u symmetrically associated with %02u\n", joint_id, j);
                // connect symmetric joints
                joints[joint_id].symmetric_joint = j;
                joints[j].symmetric_joint = joint_id;
                joints[j].type = JointType::normal; // enforce this option
                result = true;
                break;
            }
        }
    }
    return result;
}

void NJoint::draw(void) const
{
    const double length = 0.100; // 100mm
    const dReal *position_body_1, *position_body_2;

    dVector3 anchor, axis;
    dReal position_joint_anchor[3];
    dReal joint_axis_1[3];
    dReal joint_axis_2[3];

    dJointGetHingeAnchor(hinge, anchor);
    dJointGetHingeAxis  (hinge, axis  );

    dBodyID body1 = dJointGetBody(hinge, 0); // get id of first  body connected with 'hinge'
    dBodyID body2 = dJointGetBody(hinge, 1); // get id of second body connected with 'hinge'

    position_body_1 = dBodyGetPosition(body1);
    position_body_2 = dBodyGetPosition(body2);

    for (unsigned int i = 0; i < 3; ++i) {
        position_joint_anchor[i] = anchor[i];
        joint_axis_1[i] = anchor[i] - length * axis[i];
        joint_axis_2[i] = anchor[i] + length * axis[i];
    }

    colors::white0.apply();
    dsDrawLineD((const double *) position_body_1, (const double *) position_joint_anchor);
    dsDrawLineD((const double *) position_body_2, (const double *) position_joint_anchor);

    if (is_sticking) colors::gray0.apply();
    dsDrawLineD((const double *) joint_axis_1, (const double *) joint_axis_2);
}

/* ODE-USER GUIDE:
 * Motors can also be used to accurately model dry (or Coulomb) friction in joints.
 * Simply set the desired velocity to zero and set the maximum force to some constant
 * value - then all joint motion will be impeded by that force.
 */
void
NJoint::apply_friction(const double velocity_norm)
{
    double velocity_restore = bristle(velocity_norm);
    //printf("pos: %+.10f  vel:%+.3f  z:%+.3f  vsoll:%+.3f\n", get_motor_angle(), velocity_norm, z, velocity_restore);

    /* set max torque for friction */
    dJointSetAMotorParam(motor, dParamVel, velocity_restore);
    dJointSetAMotorParam(motor, dParamFMax, stribeck_friction_model(velocity_norm));
}

/* The BRISTLE model is a simple and numerically stable friction model inspired by
 * the LuGre friction model. It simulates the displacement of a virtual bristle
 * which is being moved over an abrasive surface. The variable z is the current
 * displacement in form of an integrator of the input velocity, hence it
 * has the form of a relative position. The integrator has a hard limiter,
 * so the bristle cannot exceed a certain length. After multiplication with a
 * proportional factor the stiffness of this bristle (like a spring) is defined.
 * The eventual restoring movement is then applied by ODE's velocity controller
 * with max force depending on the Stribeck friction model. */
double
NJoint::bristle(double velocity) //TODO: siehe Abschnitt zur kritische Dämpfung in der eigenen Studienarbeit
{
    z = z - velocity;
    z = common::clip(z, conf.bristle_displ_max);

    return z * conf.bristle_stiffness;
}

/* TODO: add a description
 *
 */
double
NJoint::stribeck_friction_model(const double velocity)
{
    double friction(.0);

    const double F_s = conf.sticking_friction;
    const double F_c = conf.coulomb_friction;
    const double F_v = conf.fluid_friction;
    const double v_s = conf.stiction_range;

    /* if velocity is in sticking range, use sticking friction */
    //friction += (fabs(velocity) < v_s)? F_s : F_c; // nicht stetig
    double range = exp(-(velocity/v_s)*(velocity/v_s));
    friction += F_c + (F_s - F_c) * range;

    /* add velocity dependent, linear increasing friction */
    friction += fabs(velocity) * F_v;

    is_sticking = (range > 0.1)? true:false;
    return friction;
}


/*-General Motor Model ----------------------------+
 |                                                 |
 | u             : input [-1,+1]                   |
 | joint_speed   : [-1,+1] (normed joint velocity) |
 | torque_factor : 5 equals 1 Dynamixel            |
 |                                                 |
 +-------------------------------------------------*/
double
NJoint::motor_model(const double u, double joint_speed) const
{
    double U_in = conf.V_in * u;

    if (common::sgn(U_in) != common::sgn(joint_speed)) joint_speed = 0; // does this fixes instability problems?

    double I = common::clip(U_in - (conf.kB * joint_speed), conf.V_in) * conf.R_i_inv; // limit to max battery voltage
    double M = conf.kM * I * (0.2*torque_factor);                            // resulting motor torque

    return M;
}
