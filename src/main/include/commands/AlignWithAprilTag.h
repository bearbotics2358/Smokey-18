#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <subsystems/CommandSwerveDrivetrain.h>

class AlignWithAprilTag : public frc2::CommandHelper<frc2::Command, AlignWithAprilTag> {
    public:
    AlignWithAprilTag(subsystems::CommandSwerveDrivetrain* drivetrain, int aprilTagId);
    
    private:
    int m_aprilTagId;
    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain()};
};