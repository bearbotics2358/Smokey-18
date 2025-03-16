
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    frc::Transform3d robotToCam = 
        frc::Transform3d(frc::Translation3d(-2.5625_in, 0_in, 12.5_in),
            frc::Rotation3d(0_rad,0_rad,0_rad));
    m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
}

//updates local variables related to the limelight result
void CameraSubsystem::updateData() {
    result = limelightCamera.GetLatestResult();
    if (result.HasTargets()) {
        bestTarget = result.GetBestTarget();
        transformation = bestTarget.GetBestCameraToTarget();
        std::optional<photon::EstimatedRobotPose> estimatedPose = m_poseEstimator->Update(result);
        if (estimatedPose){
            m_drivetrain->AddVisionMeasurement(estimatedPose->estimatedPose.ToPose2d(),ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose->timestamp));
        }

        
        frc::SmartDashboard::PutBoolean("Has Targets", true);

        frc::SmartDashboard::PutNumber("Rotation", bestTarget.GetYaw());
        frc::SmartDashboard::PutNumber("Strafe Distance", units::inch_t(getStrafeTransformation()).value());
        frc::SmartDashboard::PutNumber("Forward Distance", units::inch_t(getForwardTransformation()).value());
    } else {
        frc::SmartDashboard::PutBoolean("Has Targets", false);
    }
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    return result.HasTargets();
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

void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Camera Periodic", "Running");

    updateData();
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
