#include "sim/SimProfile.h"
#include <ctre/phoenixpro/Utils.hpp>

using namespace ctre::phoenixpro;

units::second_t SimProfile::GetPeriod() 
{
    // set the start time if not yet running
    if (!_running) {
        _lastTime = GetCurrentTimeSeconds();
        _running = true;
    }

    double now = GetCurrentTimeSeconds();
    double period = now - _lastTime;
    _lastTime = now;

    return units::second_t{period};
}