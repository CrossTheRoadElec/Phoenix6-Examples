#pragma once

#include "wpi/units/time.hpp"

/**
 * Holds information about a simulated device.
 */
class SimProfile {
    wpi::units::second_t _lastTime;
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
    wpi::units::second_t GetPeriod();
};