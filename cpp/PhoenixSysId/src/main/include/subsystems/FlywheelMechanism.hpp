#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "wpi/commands2/SubsystemBase.hpp"
#include "wpi/commands2/sysid/SysIdRoutine.hpp"

#include "Constants.hpp"

class FlywheelMechanism : public wpi::cmd::SubsystemBase {
private:
    ctre::phoenix6::hardware::TalonFX m_motorToTest{kTalonFxId, kCANbus};
    ctre::phoenix6::controls::DutyCycleOut m_joystickControl{0};
    ctre::phoenix6::controls::VoltageOut m_sysIdControl{0_V};

    wpi::cmd::sysid::SysIdRoutine m_sysIdRoutine{
        wpi::cmd::sysid::Config{
            std::nullopt, // Use default ramp rate (1 V/s)
            4_V,          // Reduce dynamic step voltage to 4 to prevent brownout
            std::nullopt, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
            [](wpi::sysid::State state)
            {
                ctre::phoenix6::SignalLogger::WriteString("state", wpi::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        wpi::cmd::sysid::Mechanism{
            [this](wpi::units::volt_t volts) { m_motorToTest.SetControl(m_sysIdControl.WithOutput(volts)); },
            {},
            this
        }
    };

public:
    FlywheelMechanism();

    wpi::cmd::CommandPtr JoystickDriveCommand(std::function<double()> output);

    wpi::cmd::CommandPtr SysIdQuasistatic(wpi::cmd::sysid::Direction direction);
    wpi::cmd::CommandPtr SysIdDynamic(wpi::cmd::sysid::Direction direction);
};
