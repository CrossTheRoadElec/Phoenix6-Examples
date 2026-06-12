#include "subsystems/FlywheelMechanism.hpp"

using namespace ctre::phoenix6;

FlywheelMechanism::FlywheelMechanism()
{
    SetName("Flywheel");

    configs::TalonFXConfiguration cfg{};
    // Set any necessary configs in the Feedback group here
    m_motorToTest.GetConfigurator().Apply(cfg);

    /* Speed up signals for better characterization data */
    BaseStatusSignal::SetUpdateFrequencyForAll(250_Hz,
        m_motorToTest.GetPosition(),
        m_motorToTest.GetVelocity(),
        m_motorToTest.GetMotorVoltage());
    
    /* Optimize out the other signals, since they're not useful for SysId */
    m_motorToTest.OptimizeBusUtilization();

    /* Start the signal logger */
    SignalLogger::Start();
}

wpi::cmd::CommandPtr FlywheelMechanism::JoystickDriveCommand(std::function<double()> output)
{
    return Run([this, output] { m_motorToTest.SetControl(m_joystickControl.WithOutput(output())); });
}

wpi::cmd::CommandPtr FlywheelMechanism::SysIdQuasistatic(wpi::cmd::sysid::Direction direction)
{
    return m_sysIdRoutine.Quasistatic(direction);
}

wpi::cmd::CommandPtr FlywheelMechanism::SysIdDynamic(wpi::cmd::sysid::Direction direction)
{
    return m_sysIdRoutine.Dynamic(direction);
}
