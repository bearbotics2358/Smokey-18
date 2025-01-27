
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem() {
  // Implementation of subsystem constructor goes here.
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
    return result.HasTargets();
}

frc2::CommandPtr CameraSubsystem::updateData() {
    return RunOnce([this] {
        frc::SmartDashboard::PutString("shdbnksndkjdsnkjs", "sjdnkjsndkjsdnkjd");
        photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
        if (result.HasTargets()) {
            photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();
            frc::SmartDashboard::PutNumber("Yaw To Best Tag", bestTarget.GetYaw());
            frc::Transform3d pose = bestTarget.GetBestCameraToTarget();
            frc::SmartDashboard::PutNumber("X To Best Tag", pose.X().value());
            frc::SmartDashboard::PutNumber("Y To Best Tag", pose.Y().value());
            frc::SmartDashboard::PutNumber("Best Tag ID", bestTarget.GetFiducialId());
        } else {
            frc::SmartDashboard::PutNumber("Yaw To Best Tag", 0);
            frc::SmartDashboard::PutNumber("X To Best Tag", 0);
            frc::SmartDashboard::PutNumber("Y To Best Tag", 0);
            frc::SmartDashboard::PutNumber("Best Tag ID", 0);
        }
    });
}

void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("please work", "work work work");
    photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
    if (result.HasTargets()) {
        photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();
        frc::SmartDashboard::PutNumber("Yaw To Best Tag", bestTarget.GetYaw());
        frc::Transform3d pose = bestTarget.GetBestCameraToTarget();
        frc::SmartDashboard::PutNumber("X To Best Tag", pose.X().value());
        frc::SmartDashboard::PutNumber("Y To Best Tag", pose.Y().value());
        frc::SmartDashboard::PutNumber("Best Tag ID", bestTarget.GetFiducialId());
    } else {
        frc::SmartDashboard::PutNumber("Yaw To Best Tag", 0);
        frc::SmartDashboard::PutNumber("X To Best Tag", 0);
        frc::SmartDashboard::PutNumber("Y To Best Tag", 0);
        frc::SmartDashboard::PutNumber("Best Tag ID", 0);
    }
}
