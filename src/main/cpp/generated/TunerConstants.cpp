#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain(std::function<void()> AddPathPlannerCommands)
{
    std::function<void()> m_addPathPlannerCommands = std::function<void()>(AddPathPlannerCommands);
    if (m_addPathPlannerCommands) {
        m_addPathPlannerCommands();
    }
    
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
