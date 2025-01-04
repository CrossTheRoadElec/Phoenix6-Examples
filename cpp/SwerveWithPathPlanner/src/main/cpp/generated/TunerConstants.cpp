#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
