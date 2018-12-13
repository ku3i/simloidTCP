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
        printf("Using standard actuator parameters.");
        static_assert (joint::sticking_friction >= joint::coulomb_friction, "Stiction must be greater than coulomb friction.");
        static_assert (joint::stiction_range > 0, "Stiction range must be greater zero.");
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
        printf("Using randomized actuator parameters by %1.2f %% variation", perc * 100 * var);
        assert (perc >= 0. and perc <= 0.33);
        assert (var  >= 0. and var  <= 1.00);
        /* Stiction must be greater than coulomb friction. This is guarantied up to a range of 33%*/
        assert (sticking_friction >= coulomb_friction);
        assert (stiction_range > 0);
    }

    ActuatorParameters(std::vector<double> params, bool assert_range = true)
    : bristle_displ_max( params.at(0) )
    , bristle_stiffness( params.at(1) )
    , sticking_friction( params.at(2) )
    , coulomb_friction ( params.at(3) )
    , fluid_friction   ( params.at(4) )
    , stiction_range   ( params.at(5) )
    , V_in             ( params.at(6) )
    , kB               ( params.at(7) )
    , kM               ( params.at(8) )
    , R_i_inv          ( params.at(9) )
    {
        printf("Using actuator parameters from vector.\n"
               "bristle_displ_max : %7.4lf | bristle_stiffness : %7.4lf\n"
               "sticking_friction : %7.4lf | coulomb_friction  : %7.4lf\n"
               "fluid_friction    : %7.4lf | stiction_range    : %7.4lf\n"
               "V_in              : %7.4lf | kB                : %7.4lf\n"
               "kM                : %7.4lf | R_i_inv           : %7.4lf\n"
              , bristle_displ_max
              , bristle_stiffness
              , sticking_friction
              , coulomb_friction
              , fluid_friction
              , stiction_range
              , V_in
              , kB
              , kM
              , R_i_inv
              );
        if (assert_range) {
            assert (sticking_friction >= coulomb_friction);
            assert (stiction_range > 0);
        }
    }

    std::vector<double> get(void) const
    {
        std::vector<double> params;
        params.reserve(10);
        params.emplace_back( bristle_displ_max );
        params.emplace_back( bristle_stiffness );
        params.emplace_back( sticking_friction );
        params.emplace_back( coulomb_friction  );
        params.emplace_back( fluid_friction    );
        params.emplace_back( stiction_range    );
        params.emplace_back( V_in              );
        params.emplace_back( kB                );
        params.emplace_back( kM                );
        params.emplace_back( R_i_inv           );
        assert(params.size() == 10);
        return params;
    }

    void randomize(double perc, double var)
    {
        if (perc == .0 or var == .0) { printf("no randomization performed in actuator parameters."); return; }

        bristle_displ_max = rnd(bristle_displ_max, perc, var);
        bristle_stiffness = rnd(bristle_stiffness, perc, var);
        sticking_friction = rnd(sticking_friction, perc, var);
        coulomb_friction  = rnd(coulomb_friction , perc, var);
        fluid_friction    = rnd(fluid_friction   , perc, var);
        stiction_range    = rnd(stiction_range   , perc, var);
        V_in              = rnd(V_in             , perc, var);
        kB                = rnd(kB               , perc, var);
        kM                = rnd(kM               , perc, var);
        R_i_inv           = rnd(R_i_inv          , perc, var);

        printf("Randomizing current actuator parameters by %1.2f %% variation", perc * 100 * var);
        assert (perc >= 0. and perc <= 0.33);
        assert (var  >= 0. and var  <= 1.00);
        /* Stiction must be greater than coulomb friction. This is guarantied up to a range of 33%*/
        if (sticking_friction <= coulomb_friction)
            sticking_friction = 1.01 * coulomb_friction;
        assert (stiction_range > 0);
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
