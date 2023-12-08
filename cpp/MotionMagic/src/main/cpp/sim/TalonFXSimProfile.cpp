#include "sim/TalonFXSimProfile.h"

using namespace ctre::phoenix6;

TalonFXSimProfile::TalonFXSimProfile(hardware::TalonFX& falcon, units::kilogram_square_meter_t rotorInertia) :
    _motorSim{frc::DCMotor::Falcon500FOC(1), 1, rotorInertia},
    _falcon{falcon} {
}

void TalonFXSimProfile::Run() {
    /// DEVICE SPEED SIMULATION

    _motorSim.SetInputVoltage(_falcon.GetSimState().GetMotorVoltage());

    _motorSim.Update(GetPeriod());

    /// SET SIM PHYSICS INPUTS
    auto const position = _motorSim.GetAngularPosition();
    auto const velocity = _motorSim.GetAngularVelocity();

    _falcon.GetSimState().SetRawRotorPosition(position);
    _falcon.GetSimState().SetRotorVelocity(velocity);

    _falcon.GetSimState().SetSupplyVoltage(12_V - _falcon.GetSimState().GetSupplyCurrent() * kMotorResistance);
}