#pragma once

#include "sim/SimProfile.h"

#include <ctre/phoenixpro/TalonFX.hpp>
#include <frc/simulation/DCMotorSim.h>

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile : public SimProfile {
    units::ohm_t kMotorResistance = 2_mOhm; // Assume 2mOhm resistance for voltage drop calculation
    frc::sim::DCMotorSim _motorSim;
    ctre::phoenixpro::hardware::TalonFX& _falcon;


public:
    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param falcon
     *                        The TalonFX device
     * @param rotorInertia
     *                        Rotational inertia at the Rotor
     */
    TalonFXSimProfile(ctre::phoenixpro::hardware::TalonFX& falcon, units::kilogram_square_meter_t rotorInertia);

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    void Run();
};