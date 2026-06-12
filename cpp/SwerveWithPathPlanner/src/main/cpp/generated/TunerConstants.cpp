#include "generated/TunerConstants.hpp"
#include "subsystems/CommandSwerveDrivetrain.hpp"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
