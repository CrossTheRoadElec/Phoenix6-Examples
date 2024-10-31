#include "sim/SimProfile.h"
#include <ctre/phoenix6/Utils.hpp>

using namespace ctre::phoenix6;

units::second_t SimProfile::GetPeriod() 
{
    // set the start time if not yet running
    if (!_running) {
        _lastTime = utils::GetCurrentTime();
        _running = true;
    }

    auto now = utils::GetCurrentTime();
    auto period = now - _lastTime;
    _lastTime = now;

    return period;
}