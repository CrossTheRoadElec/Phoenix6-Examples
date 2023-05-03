#include "sim/TalonFXSimProfile.h"

using namespace ctre::phoenixpro;

TalonFXSimProfile::TalonFXSimProfile(hardware::TalonFX& falcon, units::kilogram_square_meter_t rotorInertia) :
    _motorSim{frc::DCMotor::Falcon500(1), 1, rotorInertia},
    _falcon{falcon} {
}

void TalonFXSimProfile::Run() {
    /// DEVICE SPEED SIMULATION

    _motorSim.SetInputVoltage(_falcon.GetSimState().GetMotorVoltage());

    _motorSim.Update(GetPeriod());

    /// SET SIM PHYSICS INPUTS
    auto velocity = _motorSim.GetAngularVelocity();

    _falcon.GetSimState().SetRawRotorPosition(_motorSim.GetAngularPosition());
    _falcon.GetSimState().SetRotorVelocity(velocity);

    _falcon.GetSimState().SetSupplyVoltage(12_V - _falcon.GetSimState().GetSupplyCurrent() * kMotorResistance);
}