#pragma once

#include <ctre/phoenix6/SignalLogger.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include "Constants.h"

class FlywheelMechanism : public frc2::SubsystemBase
{
private:
    ctre::phoenix6::hardware::TalonFX m_motorToTest{kTalonFxId, kCANbus};
    ctre::phoenix6::controls::DutyCycleOut m_joystickControl{0};
    ctre::phoenix6::controls::VoltageOut m_sysIdControl{0_V};

    frc2::sysid::SysIdRoutine m_sysIdRoutine{
        frc2::sysid::Config{
            std::nullopt, // Use default ramp rate (1 V/s)
            4_V,          // Reduce dynamic step voltage to 4 to prevent brownout
            std::nullopt, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
            [](frc::sysid::State state)
            {
                ctre::phoenix6::SignalLogger::WriteString("state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t volts) { m_motorToTest.SetControl(m_sysIdControl.WithOutput(volts)); },
            {},
            this
        }
    };

public:
    FlywheelMechanism();

    frc2::CommandPtr JoystickDriveCommand(std::function<double()> output);

    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);
};
