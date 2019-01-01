#ifndef JOINTS_H_INCLUDED
#define JOINTS_H_INCLUDED

#include <string>
#include <vector>
#include <ode/ode.h>

#include <draw/drawstuff.h>
#include <basic/common.h>
#include <basic/color.h>
#include <basic/configuration.h>
#include <basic/derivative.h>
#include <build/bodies.h>
#include <build/params.h>
#include <controller/pid_controller.h>

extern Configuration global_conf;

enum JointType {normal, symmetric};

class NJoint
{
public:

    NJoint( const dWorldID &world
          , const SolidVector& bodies
          , const unsigned int joint_id
          , const unsigned int body1
          , const unsigned int body2
          , const JointType type
          , std::string name
          , double stop_lo_rad
          , double stop_hi_rad
          , double position_default_rad
          , Vector3 rel
          , const char axis
          , double torque_factor
          , ActuatorParameters const& conf
          )
    : joint_id(joint_id)
    , body1(body1)
    , body2(body2)
    , torque_factor(torque_factor)
    , symmetric_joint(joint_id)
    , type(type)
    , name(name)
    , stop_lo(common::rad2norm(stop_lo_rad))
    , stop_hi(common::rad2norm(stop_hi_rad))
    , position_default(common::rad2norm(position_default_rad))
    , pid_ctrl(global_conf.pidP, global_conf.pidI, global_conf.pidD, -0.5, 0.5)
    , pid_enable(false)
    , pid_maxtorque(global_conf.init_max_torque)
    , pid_position_setpoint(position_default)
    , voltage_setpoint(0.0)
    , is_sticking(false)
    , z(.0)
    , conf(conf)
    //, dpdt(.0, global_conf.step_length, /*scale=*/0.25, /*change_limit=*/0.1)
    , dpdt(.0, global_conf.step_length, /*scale=*/0.25)
    {
        if (name == "") {
            name = "joint_" + std::to_string(joint_id);
            dsPrint("Warning: No name assigned to joint no. %u\n", joint_id);
        }

        dsPrint("Creating new joint number %d with name '%s'.\n", joint_id, name.c_str());

        if ((stop_lo > position_default) || (position_default > stop_hi))
            dsError("Default position out of bounds for joint %u.\n", joint_id);

        hinge = dJointCreateHinge(world, 0);
        motor = dJointCreateAMotor(world, 0);

        dJointAttach(hinge, bodies[body1].body, bodies[body2].body);
        dJointAttach(motor, bodies[body1].body, bodies[body2].body);

        unsigned int relBody = body2;
        const dReal *a = dBodyGetPosition (bodies[relBody].body);

        dJointSetHingeAnchor(hinge, a[0] + rel.x, a[1] + rel.y, a[2] + rel.z);
        dJointSetHingeAxis  (hinge, (axis == 'x') - (axis == 'X'), (axis == 'y') - (axis == 'Y'), (axis == 'z') - (axis == 'Z'));

        dJointSetAMotorMode (motor, dAMotorEuler);

        dJointSetAMotorAxis (motor, 0, 1, (axis=='x') - (axis=='X'), (axis=='y') - (axis=='Y'), (axis=='z') - (axis=='Z'));
        dJointSetAMotorAxis (motor, 2, 1, (axis=='y') - (axis=='Y'), (axis=='z') - (axis=='Z'), (axis=='x') - (axis=='X'));

        dJointSetAMotorParam(motor, dParamCFM,  constants::joint::cfm);

        /* initial static friction*/
        apply_friction(0.0);

        // set joint limits
        if (stop_lo == -1 && stop_hi == 1) {
            dsPrint("Joint stops deactivated for joint %u.\n", joint_id);
        }
        else if (stop_lo < stop_hi)
        {
            // set stops
            dJointSetHingeParam (hinge, dParamLoStop, stop_lo_rad);
            dJointSetHingeParam (hinge, dParamHiStop, stop_hi_rad);
            dJointSetAMotorParam(motor, dParamLoStop, stop_lo_rad);
            dJointSetAMotorParam(motor, dParamHiStop, stop_hi_rad);
        } else
            dsError("Lower joint stop is greater than higher (%1.2f > %1.2f)\n", common::rad2deg(stop_lo_rad), common::rad2deg(stop_hi_rad));

        assert(torque_factor > 0. and torque_factor <= 10);
        dsPrint("done.\n");

        dpdt.reset(get_low_resolution_position(false));
    }

    ~NJoint()
    {
        dsPrint("Destroying joint number %2u...", joint_id);
        dJointDestroy(hinge);
        dJointDestroy(motor);
        dsPrint("done.\n");
    }

    /* lower resolution and noisy sensor outputs */
    double get_low_resolution_position(bool low_quality) const {
        return low_quality ? common::avr_10bit_adc(get_position_norm())
                           : common::low_resolution_sensor(get_position_norm());
    }

    double get_low_resolution_velocity(bool low_quality) {
        dpdt.derive(get_low_resolution_position(low_quality));
        return low_quality ? dpdt.get()
                           : common::low_resolution_sensor(get_velocity_norm());
    }

    /* get */
    double get_position_norm()  const { return common::rad2norm(dJointGetHingeAngle    (hinge)); } // +/-pi   --> +/-1
    double get_velocity_norm()  const { return common::vel2norm(dJointGetHingeAngleRate(hinge)); } // +/-2rps --> +/-1

    double get_motor_angle()    const { return dJointGetAMotorAngle(motor, 0); }

    /* set */
    void set_voltage(const double value) {
        pid_enable = false;
        voltage_setpoint = motor_model(common::clip(value, 1.0), get_velocity_norm());
    }
    void set_position(const double value) {
        pid_enable = true;
        pid_position_setpoint = common::clip(value, 1.0) * M_PI;
    }
    void set_pidmaxtorque(const double value) {
        pid_maxtorque = common::clip(value, 0.0, 1.0);
    }
    void set_motor_angular_velocity(double value) const {
        dJointSetAMotorParam(motor, dParamVel, value);
    }

    /* apply */
    void apply_voltage_control() const {
        dJointAddAMotorTorques(motor, voltage_setpoint, .0, .0);
    }
    void apply_pidmaxtorque() const {
        dJointSetAMotorParam(motor, dParamFMax, motor_model(pid_maxtorque, 0.0));
    }
    void apply_position_control() {
        set_motor_angular_velocity(pid_ctrl.set_position(pid_position_setpoint, get_motor_angle()));
    }
    void apply_friction(const double velocity_norm);

    void apply_control() {
        if (pid_enable) {
            apply_pidmaxtorque();
            apply_position_control();
        }
        else {
            apply_friction(get_velocity_norm());
            apply_voltage_control();
        }
    }

    /* physics simulation */
    double bristle(double velocity);
    double stribeck_friction_model(const double velocity);
    double motor_model(const double u, const double joint_speed) const;

    void reset() {
        pid_ctrl.reset();
        set_motor_angular_velocity(0.0);
        pid_enable = false;
        voltage_setpoint = 0.0;
        pid_position_setpoint = position_default;
        pid_maxtorque = common::clip(global_conf.init_max_torque, 0.0, 1.0);
        dpdt.reset(get_low_resolution_position(false));
    }

    void reinit_motormodel(ActuatorParameters const& c) { conf = c; }

    void draw(void) const;

    const unsigned int joint_id;
    const unsigned int body1;
    const unsigned int body2;
    const double       torque_factor;
          unsigned int symmetric_joint;
             JointType type;
    std::string        name;
    double             stop_lo;
    double             stop_hi;
    double             position_default;

private:
    dJointID           hinge;
    dJointID           motor;

    PIDController      pid_ctrl;

    bool               pid_enable;
    double             pid_maxtorque;
    double             pid_position_setpoint;

    double             voltage_setpoint;

    bool               is_sticking;

    double             z; // Bristle displacement for friction model

    ActuatorParameters conf;

//    LowpassDiff        dpdt;
    Derived            dpdt;
};


class JointVector
{
public:
    JointVector(const std::size_t max_number_of_joints)
    : joints()
    , max_number_of_joints(max_number_of_joints)
    {
        dsPrint("Creating joint vector...");
        joints.reserve(max_number_of_joints);
        dsPrint("done.\n");
    }

    ~JointVector()
    {
        dsPrint("Destroying joints.\n");
    }

    unsigned int create( const dWorldID &world
                       , const SolidVector& bodies
                       , unsigned int body1
                       , unsigned int body2
                       , JointType type
                       , std::string name
                       , double stopLo_rad
                       , double stopHi_rad
                       , double position_default_rad
                       , Vector3 rel
                       , const char axis
                       , double torque_factor
                       , ActuatorParameters const& conf
                       )
    {
        unsigned int joint_id = joints.size();
        if (joint_id < max_number_of_joints)
            joints.emplace_back( world, bodies, joint_id, body1, body2, type, name
                               , stopLo_rad, stopHi_rad, position_default_rad, rel
                               , axis, torque_factor, conf );
        else
            dsError("Maximum number of joints is %u.", max_number_of_joints);

        return joint_id;
    }

    bool add_symmetric(std::size_t joint_id, std::string name);

          NJoint& operator[](std::size_t idx)       { return joints[idx]; }
    const NJoint& operator[](std::size_t idx) const { return joints[idx]; }

    std::size_t get_size() const { return joints.size(); }

    void reset_all(void) {
        for (unsigned int idx = 0; idx < get_size(); ++idx)
            joints[idx].reset();
    }

    void apply_control_all(void) {
        for (unsigned int idx = 0; idx < get_size(); ++idx)
            joints[idx].apply_control();
    }

    void destroy(void) {
        dsPrint("Destroying joints (for recreation).\n");
        joints.clear();
    }

private:
    std::vector<NJoint> joints;
    const std::size_t   max_number_of_joints;

};


dJointID create_fixed_joint(dWorldID const& world, SolidVector const& bodies, unsigned body1, unsigned body2);

#endif // JOINTS_H_INCLUDED
