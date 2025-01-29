
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
        double x = pose.X().value();
        double y = pose.Y().value();
        double rot = pose.Rotation().Z().value();
        frc::SmartDashboard::PutNumber("X To Best Tag", x);
        frc::SmartDashboard::PutNumber("Y To Best Tag", y);
        frc::SmartDashboard::PutNumber("Rotation To Best Tag", rot);
        frc::SmartDashboard::PutNumber("Best Tag ID", bestTarget.GetFiducialId());

        double tagAngle;
        switch (bestTarget.GetFiducialId()){
            case 7:
            case 10:
            case 18:
            case 21:
                tagAngle = M_PI / 2;
                break;
            case 17:
            case 20:
            case 11:
            case 8:
                tagAngle = M_PI / 3;
                break;
            case 19:
            case 22:
            case 9:
            case 6:
                tagAngle = 2 * M_PI / 3;
                break;
        }

        //turn rot degrees
        //move parallel to tag until yaw = 0
        //raise elevator
        //move perpendicular to tag until ready to score


    } else {
        frc::SmartDashboard::PutNumber("Yaw To Best Tag", 0);
        frc::SmartDashboard::PutNumber("X To Best Tag", 0);
        frc::SmartDashboard::PutNumber("Y To Best Tag", 0);
        frc::SmartDashboard::PutNumber("Best Tag ID", 0);
    }
}
