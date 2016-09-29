#include <controller/pid_controller.h>

double PIDController::calc_velocity(const double setpoint, const double current_angle)
{
    double pTerm, dTerm, iTerm;
    const double presentError = setpoint - current_angle;

    // calculate the proportional term
    pTerm = pGain * presentError;

    // calculate the integral state with appropriate limiting
    iState += presentError;
    if (iState > iMax) iState = iMax;
    else if (iState < iMin) iState = iMin;

    // calculate the integral term
    iTerm = iGain * iState;

    // calculate the differential term
    dTerm = dGain * (presentError - lastError);
    lastError = presentError;

    return pTerm + iTerm - dTerm;
}

double PIDController::set_position(const double setpoint, const double current_angle)
{
    double velocity;

    if (setpoint > constants::m_pi) velocity = calc_velocity(setpoint - 2*constants::m_pi, current_angle);
    else if (setpoint > -constants::m_pi) velocity = calc_velocity(setpoint, current_angle);
    else velocity = calc_velocity(setpoint + 2*constants::m_pi, current_angle);

    return velocity;
}

void PIDController::reset()
{
    lastError = 0.0;
    iState = 0.0;
}
