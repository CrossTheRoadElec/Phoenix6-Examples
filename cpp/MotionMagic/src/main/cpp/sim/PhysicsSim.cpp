#include "sim/PhysicsSim.hpp"
#include "sim/TalonFXSimProfile.hpp"

using namespace ctre::phoenix6;

void PhysicsSim::AddTalonFX(hardware::TalonFX &talonFX, wpi::units::kilogram_square_meter_t rotorInertia)
{
    _simProfiles.push_back(std::make_unique<TalonFXSimProfile>(talonFX, rotorInertia));
}

void PhysicsSim::Run()
{
    // Simulate devices
    for (auto const &simProfile : _simProfiles) {
        simProfile->Run();
    }
}
