#ifndef _DEFINES_H_
#define _DEFINES_H_

#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD

namespace constants {
    const double m_pi = 3.141592653589793238512808959406186204433;
    const double h_pi = 1.570796326794896619256404479703093102216; // halved pi

    const double gravity = 9.81;
    const double range_accelsensor = 8.0; // 8G, e.g. MMA7455L

    const double default_step_length = 0.01;
    const double contact_soft_ERP    = 0.30;
    const double contact_soft_CFM    = 0.0001;

    const double world_CFM = 1e-5;
    const double world_ERP = 0.20;

    namespace friction
    {
        const double lo     =   1.0;
        const double modest =   5.0;
        const double normal =  10.0;
        const double hi     =  40.0;
        const double sticky = 100.0;
    }

    namespace joint
    {
        /* These values have been evaluated with care to have
         * realistic behavior on all the simloid robots created
         * so far. Don't change them recklessly. */
        const double fluid_friction    = 0.1;
        constexpr double coulomb_friction  = 0.1;
        constexpr double sticking_friction = 2 * coulomb_friction;
        constexpr double stiction_range    = 0.01; // 1% of normed velocity

        /* bristle model */
        const double bristle_displ_max = 0.01;
        const double bristle_stiffness = 1.0;//10.0; //TODO can we get rid of this?


        const double cfm = 0.1;   // constraint force mixing, damping factor of a soft joint constraint
    }

    namespace sensor_noise
    {
        const double std_dev = 16.0/32768.0; // 4bit std deviation
        const double max_val = 64.0/32768.0;
        const double min_val = -max_val;
    }

    namespace avr_adc_10bit
    {
        const double std_dev = 1.0/4096.0;
        const double max_val = 4.0/4096.0;
        const double min_val = -max_val;
        const double v_scale = 0.2;
    }

    const unsigned int max_bodies = 32; // max. number of bodies
    const unsigned int max_joints = 32; // max. number of joints
    const unsigned int max_accels = 32; // max. number of acceleration sensors

    const unsigned int max_obstacles = 1000;
    const unsigned int max_heightfields = 8;

    const double max_position = 100.0;   // largest allowed body distance from center at creation time
    const unsigned int max_contacts = 4; // maximum number of contact points per body/collision

    /* motor model */
    namespace motor_parameter
    {
        const double kM = 1.393;         // torque constant
        const double kB = 2.563;         // speed constant
        const double R_i_inv = 1.0/9.59; // coil resistance (inverted)
        const double V_in = 16.0;        // input voltage of the motor

        /* Estimation of max. angular velocity:
         * Datasheet of Dynamixel RX-28 states a max speed of 0.126 sec/60° at 16V.
         * This is 7.937 * 60°/sec = 476.19 °/sec.
         * A circle has 360°, so this is 1.323 RPS (rounds per sec).
         * A full turn is 2 pi, so max speed is 8.311 rad/s
         * With an appropriate amount of headroom, we allow a max speed of 2 RPS
         * This is 2*2*pi =
         */
         const double max_joint_vel = 4.0 * m_pi; // 2 RPS ~ 12 rad/s

    }

    namespace camera_parameters
    {
        const double rate_turn         = 1.0 * m_pi/180.0; // 1 DEG
        const double rate_turn_inc     = 0.01;
        const double rate_zoom_in      = 0.9;
        const double rate_zoom_out     = 1.0/rate_zoom_in;
        const double fade_rotation     = 0.01;
        const double fade_translation  = 0.25;
        const double rotation_velocity = 0.01;
    }

    namespace materials // all densities measured in kg/m^3
    {
        const double light  =  200.0; // balsa wood
        const double normal =  500.0; // spruce wood
        const double body   = 1000.0; // approx. density of a human body or water
        const double heavy  = 1500.0;
        const double rock   = 2800.0; // density of stone (granite)
    }
}

#endif
