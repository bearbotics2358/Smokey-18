
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    frc::Transform3d robotToCam =
        frc::Transform3d(frc::Translation3d(-2.5625_in, 0_in, 12.5_in),
            frc::Rotation3d(0_rad, 15_deg, 0_rad));
}

//updates local variables related to the limelight result
void CameraSubsystem::updateData() {
    if (visibleTargets()) {

        std::vector<double> botPose = nt::NetworkTableInstance::GetDefault().GetTable(kLimelight4)->GetNumberArray("botpose",std::vector<double>(6));
        frc::SmartDashboard::PutNumber("Tag Distance", units::inch_t(botPose[9]).value());

        std::vector<double> targetPose = LimelightHelpers::getBotpose_TargetSpace(kLimelight4);

        transformation = frc::Transform3d(units::meter_t(botPose[9]),
                                          units::meter_t(targetPose.at(0)),
                                          units::meter_t(targetPose.at(2)),
                                          frc::Rotation3d(units::angle::radian_t(targetPose.at(5)),
                                                          units::angle::radian_t(targetPose.at(4)),
                                                          units::angle::radian_t(targetPose.at(3))));

        LimelightHelpers::PoseEstimate estimatedPose = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelight4);
        m_drivetrain->AddVisionMeasurement(estimatedPose.pose, ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose.timestampSeconds));

        frc::SmartDashboard::PutNumber("Strafe Distance", units::inch_t(getStrafeTransformation()).value());
        frc::SmartDashboard::PutNumber("Forward Distance", units::inch_t(getForwardTransformation()).value());
    }

    frc::SmartDashboard::PutBoolean("Has Targets", visibleTargets());
}

std::optional<int> CameraSubsystem::GetTargetTagId() {
    if (visibleTargets()) {
        return LimelightHelpers::getFiducialID(kLimelight4);
    } else {
        return std::nullopt;
    }
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    return LimelightHelpers::getTV(kLimelight4);
}

units::meter_t CameraSubsystem::getStrafeTransformation() {
    return transformation.Y();
}

units::meter_t CameraSubsystem::getForwardTransformation() {
    return transformation.X();
}

frc::Rotation2d CameraSubsystem::GetRotation2d() {
    return transformation.Rotation().ToRotation2d();
}

void CameraSubsystem::Periodic() {
    updateData();

    frc::SmartDashboard::PutNumber("Raw Forward Transformation", LimelightHelpers::getBotpose_TargetSpace(kLimelight4).at(1) * 39.37);
    frc::SmartDashboard::PutNumber("Robot X Position", m_drivetrain->GetPose().X().value());
    frc::SmartDashboard::PutNumber("Robot Y Position", m_drivetrain->GetPose().Y().value());
    frc::SmartDashboard::PutNumber("Robot Rotation", m_drivetrain->GetPose().Rotation().Degrees().value());
}
