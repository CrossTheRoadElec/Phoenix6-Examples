#include "sim/TalonFXSimProfile.hpp"
#include "wpi/math/system/Models.hpp"

using namespace ctre::phoenix6;

TalonFXSimProfile::TalonFXSimProfile(hardware::TalonFX &talonFX, wpi::units::kilogram_square_meter_t rotorInertia) :
    _motorSim{
        wpi::math::Models::SingleJointedArmFromPhysicalConstants(wpi::math::DCMotor::KrakenX60FOC(1), rotorInertia, 1),
        wpi::math::DCMotor::KrakenX60FOC(1)
    },
    _talonFXSim{talonFX.GetSimState()}
{}

void TalonFXSimProfile::Run() {
    /// DEVICE SPEED SIMULATION

    _motorSim.SetInputVoltage(_talonFXSim.GetMotorVoltage());

    _motorSim.Update(GetPeriod());

    /// SET SIM PHYSICS INPUTS
    auto const position = _motorSim.GetAngularPosition();
    auto const velocity = _motorSim.GetAngularVelocity();

    _talonFXSim.SetRawRotorPosition(position);
    _talonFXSim.SetRotorVelocity(velocity);

    _talonFXSim.SetSupplyVoltage(12_V - _talonFXSim.GetSupplyCurrent() * kMotorResistance);
}