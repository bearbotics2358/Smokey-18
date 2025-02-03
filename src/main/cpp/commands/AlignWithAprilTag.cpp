#include <commands/AlignWithAprilTag.h>

AlignWithAprilTag::AlignWithAprilTag(subsystems::CommandSwerveDrivetrain* drivetrain, int aprilTagId) {
    m_aprilTagId = aprilTagId;
    AddRequirements(drivetrain);
}