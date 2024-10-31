#include "sim/PhysicsSim.h"
#include "sim/TalonFXSimProfile.h"

using namespace ctre::phoenix6;

void PhysicsSim::AddTalonFX(hardware::TalonFX& talonFX, units::kilogram_square_meter_t rotorInertia) {
    TalonFXSimProfile *simTalonFX = new TalonFXSimProfile(talonFX, rotorInertia);
    _simProfiles.push_back(simTalonFX);
}

void PhysicsSim::Run() {
    // Simulate devices
    for (auto simProfile : _simProfiles) {
        simProfile->Run();
    }
}
