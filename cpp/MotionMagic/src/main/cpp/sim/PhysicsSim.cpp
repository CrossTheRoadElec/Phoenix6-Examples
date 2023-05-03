#include "sim/PhysicsSim.h"
#include "sim/TalonFXSimProfile.h"

using namespace ctre::phoenixpro;

void PhysicsSim::AddTalonFX(hardware::TalonFX& falcon, units::kilogram_square_meter_t rotorInertia) {
    TalonFXSimProfile *simFalcon = new TalonFXSimProfile(falcon, rotorInertia);
    _simProfiles.push_back(simFalcon);
}

void PhysicsSim::Run() {
    // Simulate devices
    for (auto simProfile : _simProfiles) {
        simProfile->Run();
    }
}
