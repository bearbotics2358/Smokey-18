
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    frc::Transform3d robotToCam =
        frc::Transform3d(frc::Translation3d(-2.5625_in, 0_in, 12.5_in),
            frc::Rotation3d(0_rad, 15_deg, 0_rad));
    // m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
}

//updates local variables related to the limelight result
void CameraSubsystem::updateData() {
    if (visibleTargets()) {
        // @todo The Limelight documentation indicates this call is somewhat slow and is not recommended as the way
        //       to retrieve information from the camera. This should be changed to get some of the raw data instead.
        LimelightHelpers::LimelightResultsClass results = LimelightHelpers::getLatestResults(kLimelight4);

        if (results.targetingResults.valid) {
            // @todo Choose the best target from the list of targetingResults

            //bestTarget = result.GetBestTarget();
            //transformation = bestTarget.GetBestCameraToTarget();
            LimelightHelpers::PoseEstimate estimatedPose = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelight4);
            m_drivetrain->AddVisionMeasurement(estimatedPose.pose,ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose.timestampSeconds));
        }

        frc::SmartDashboard::PutBoolean("Has Targets", true);

        //frc::SmartDashboard::PutNumber("Rotation", bestTarget.GetYaw());
        frc::SmartDashboard::PutNumber("Strafe Distance", units::inch_t(getStrafeTransformation()).value());
        frc::SmartDashboard::PutNumber("Forward Distance", units::inch_t(getForwardTransformation()).value());
        frc::SmartDashboard::PutNumber("Distance", units::inch_t(getDistance()).value());
    } else {
        frc::SmartDashboard::PutBoolean("Has Targets", false);
    }
}

std::optional<int> CameraSubsystem::GetTargetTagId() {
    if (visibleTargets()) {
        return bestTarget.GetFiducialId();
    } else {
        return std::nullopt;
    }
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    return LimelightHelpers::getTV(kLimelight4);
}

//returns the Z rotation needed to get to the best target as a double
units::degree_t CameraSubsystem::getZRotation() {
    if (result.HasTargets()) {
        return units::degree_t(bestTarget.GetYaw());
    } else {
        return 0_deg;
    }
}

// meters
double CameraSubsystem::getDistance() {
    if (result.HasTargets()) {
        return sqrt(pow(transformation.Y().value(), 2) + pow(transformation.X().value(), 2));
    } else {
        return 0;
    }
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
    frc::SmartDashboard::PutString("Camera Periodic", "Running");

    updateData();

    frc::SmartDashboard::PutNumber("Robot X Position", m_drivetrain->GetPose().X().value());
    frc::SmartDashboard::PutNumber("Robot Y Position", m_drivetrain->GetPose().Y().value());
    frc::SmartDashboard::PutNumber("Robot Rotation", m_drivetrain->GetPose().Rotation().Degrees().value());
}
