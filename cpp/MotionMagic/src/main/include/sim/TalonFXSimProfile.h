#pragma once

#include "sim/SimProfile.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/simulation/DCMotorSim.h>

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile : public SimProfile {
    static constexpr units::ohm_t kMotorResistance = 2_mOhm; // Assume 2mOhm resistance for voltage drop calculation
    frc::sim::DCMotorSim _motorSim;
    ctre::phoenix6::sim::TalonFXSimState& _talonFXSim;


public:
    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param talonFX
     *                        The TalonFX device
     * @param rotorInertia
     *                        Rotational inertia at the Rotor
     */
    TalonFXSimProfile(ctre::phoenix6::hardware::TalonFX& talonFX, units::kilogram_square_meter_t rotorInertia);

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    void Run();
};