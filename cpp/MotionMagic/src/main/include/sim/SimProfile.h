#pragma once

#include <units/time.h>

/**
 * Holds information about a simulated device.
 */
class SimProfile {
    double _lastTime;
    bool _running = false;

public:
    /**
     * Runs the simulation profile.
     * Implemented by device-specific profiles.
     */
    virtual void Run() = 0;

protected:
    /**
     * Returns the time since last call, in seconds.
     */
    units::second_t GetPeriod();
};