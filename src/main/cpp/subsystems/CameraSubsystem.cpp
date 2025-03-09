
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem() {
  // Implementation of subsystem constructor goes here.
}

//updates local variables related to the limelight result
void CameraSubsystem::updateData() {
    result = limelightCamera.GetLatestResult();
    if (result.HasTargets()) {
        bestTarget = result.GetBestTarget();
        transformation = bestTarget.GetBestCameraToTarget();
        frc::SmartDashboard::PutBoolean("Has Targets", true);
    } else {
        frc::SmartDashboard::PutBoolean("Has Targets", false);
    }
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    CameraSubsystem::updateData();
    return result.HasTargets();
}

//returns the Z rotation needed to get to the best target as a double
double CameraSubsystem::getZRotation() {
    CameraSubsystem::updateData();
    if (result.HasTargets()) {
        return fabs((transformation.Rotation().Z().value()) - M_PI) * (bestTarget.GetYaw() / fabs(bestTarget.GetYaw()));
    } else {
        return 0;
    }
}

//returns the Y translation needed to get to the best target as a double
double CameraSubsystem::getYDistance() {
    CameraSubsystem::updateData();
    if (result.HasTargets()) {
        return transformation.Y().value();
    } else {
        return 0;
    }
}

// meters 
double CameraSubsystem::getDistance() {
    CameraSubsystem::updateData();
    if (result.HasTargets()) {
        return sqrt(pow(transformation.Y().value(), 2) + pow(transformation.X().value(), 2));
    } else {
        return 0;
    }
}

//meters
double CameraSubsystem::getHorizontalTransformation() {
    CameraSubsystem::updateData();
    double distance = CameraSubsystem::getDistance();;
    return cos(getZRotation()) * distance;
}

//meters
double CameraSubsystem::getForwardTransformation() {
    CameraSubsystem::updateData();
    double distance = CameraSubsystem::getDistance();
    return sin(getZRotation()) * distance;
}

void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Camera Periodic", "Running");
    // photon::PhotonPipelineResult result = limelightCamera.GetLatestResult();
    // if (result.HasTargets()) {
    //     photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();
    //     frc::SmartDashboard::PutNumber("Yaw To Best Tag", bestTarget.GetYaw());
    //     frc::Transform3d pose = bestTarget.GetBestCameraToTarget();
    //     double x = pose.X().value();
    //     double y = pose.Y().value();
    //     double rotX = (((pose.Rotation().X().value())/M_PI) * 180);
    //     double rotZ = (((pose.Rotation().Z().value())/M_PI) * 180);
    //     frc::SmartDashboard::PutNumber("X To Best Tag", x);
    //     frc::SmartDashboard::PutNumber("Y To Best Tag", y);
    //     frc::SmartDashboard::PutNumber("X Rotation To Best Tag", rotX);
    //     frc::SmartDashboard::PutNumber("Z Rotation To Best Tag", rotZ);
    //     frc::SmartDashboard::PutNumber("Best Tag ID", bestTarget.GetFiducialId());
    // } else {
    //     frc::SmartDashboard::PutNumber("Yaw To Best Tag", 0);
    //     frc::SmartDashboard::PutNumber("X To Best Tag", 0);
    //     frc::SmartDashboard::PutNumber("Y To Best Tag", 0);
    //     frc::SmartDashboard::PutNumber("Best Tag ID", 0);
    // }
}
