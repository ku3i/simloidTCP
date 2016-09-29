#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <basic/common.h>
#include <basic/configuration.h>
#include <basic/constants.h>

class PIDController
{
public:
    PIDController(const double pg, const double ig, const double dg, const double aIMax, const double aIMin)
    : pGain(pg)
    , iGain(ig)
    , dGain(dg)
    , iMax(aIMax)
    , iMin(aIMin)
    , lastError(0.0)
    , iState(0.0)
    {}

    double set_position(const double setpoint, const double current_angle);
    void reset();

protected:
    const double pGain; // proportional gain
    const double iGain; // integral gain
    const double dGain; // derivative gain
    const double iMax;  // maximum allowable integrator state
    const double iMin;  // minimum allowable integrator state
    double lastError;   // last error
    double iState;      // integrator state

    double calc_velocity(const double setpoint, const double current_angle);
};

#endif
