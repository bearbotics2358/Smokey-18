
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem() {
  // Implementation of subsystem constructor goes here.
}

void CameraSubsystem::updateData() {
    photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
    if (result.HasTargets()) {
        bestTarget = result.GetBestTarget();
        transformation = bestTarget.GetBestCameraToTarget();
    } else {
        //nothing
    }
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
    return result.HasTargets();
}

double CameraSubsystem::getZRotation() {
    updateData();
    if (result.HasTargets()) {
        return transformation.Rotation().Z().value();
    }
}

double CameraSubsystem::getYDistance() {
    updateData();
    if (result.HasTargets()) {
        return transformation.Y().value();
    }
}

void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("please work", "work work work");
    photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
    if (result.HasTargets()) {
        photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();
        frc::SmartDashboard::PutNumber("Yaw To Best Tag", bestTarget.GetYaw());
        frc::Transform3d pose = bestTarget.GetBestCameraToTarget();
        double x = pose.X().value();
        double y = pose.Y().value();
        double rotX = (((pose.Rotation().X().value())/M_PI) * 180);
        double rotZ = (((pose.Rotation().Z().value())/M_PI) * 180);
        frc::SmartDashboard::PutNumber("X To Best Tag", x);
        frc::SmartDashboard::PutNumber("Y To Best Tag", y);
        frc::SmartDashboard::PutNumber("X Rotation To Best Tag", rotX);
        frc::SmartDashboard::PutNumber("Z Rotation To Best Tag", rotZ);
        frc::SmartDashboard::PutNumber("Best Tag ID", bestTarget.GetFiducialId());
    } else {
        frc::SmartDashboard::PutNumber("Yaw To Best Tag", 0);
        frc::SmartDashboard::PutNumber("X To Best Tag", 0);
        frc::SmartDashboard::PutNumber("Y To Best Tag", 0);
        frc::SmartDashboard::PutNumber("Best Tag ID", 0);
    }
}
