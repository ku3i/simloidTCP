#ifndef PARAMS_H_INCLUDED
#define PARAMS_H_INCLUDED

#include <cassert>
#include <basic/common.h>
#include <basic/constants.h>

using common::rnd;
using namespace constants;

/* This struct is for holding and randomizing all relevant motor parameters. */
struct ActuatorParameters {

    ActuatorParameters()
    : bristle_displ_max( joint::bristle_displ_max )
    , bristle_stiffness( joint::bristle_stiffness )
    , sticking_friction( joint::sticking_friction )
    , coulomb_friction ( joint::coulomb_friction  )
    , fluid_friction   ( joint::fluid_friction    )
    , stiction_range   ( joint::stiction_range    )
    , V_in             ( motor_parameter::V_in    )
    , kB               ( motor_parameter::kB      )
    , kM               ( motor_parameter::kM      )
    , R_i_inv          ( motor_parameter::R_i_inv )
    {
        dsPrint("Using standard actuator parameters.");
        static_assert (joint::sticking_friction >= joint::coulomb_friction, "Sticktion must be greater than coulomb friction.");
    }

    ActuatorParameters(double perc, double var)
    : bristle_displ_max( rnd(joint::bristle_displ_max, perc, var) )
    , bristle_stiffness( rnd(joint::bristle_stiffness, perc, var) )
    , sticking_friction( rnd(joint::sticking_friction, perc, var) )
    , coulomb_friction ( rnd(joint::coulomb_friction , perc, var) )
    , fluid_friction   ( rnd(joint::fluid_friction   , perc, var) )
    , stiction_range   ( rnd(joint::stiction_range   , perc, var) )
    , V_in             ( rnd(motor_parameter::V_in   , perc, var) )
    , kB               ( rnd(motor_parameter::kB     , perc, var) )
    , kM               ( rnd(motor_parameter::kM     , perc, var) )
    , R_i_inv          ( rnd(motor_parameter::R_i_inv, perc, var) )
    {
        dsPrint("Using randomized actuator parameters by %1.2f %% variation", perc * 100 * var);
        assert (perc >= 0. and perc <= 0.33);
        assert (var  >= 0. and var  <= 1.00);
        /* Sticktion must be greater than coulomb friction. This is guarantied up to a range of 33%*/
        assert (sticking_friction >= coulomb_friction);
    }

    double bristle_displ_max;
    double bristle_stiffness;
    double sticking_friction;
    double coulomb_friction;
    double fluid_friction;
    double stiction_range;
    double V_in;
    double kB;
    double kM;
    double R_i_inv;
};

#endif // PARAMS_H_INCLUDED
