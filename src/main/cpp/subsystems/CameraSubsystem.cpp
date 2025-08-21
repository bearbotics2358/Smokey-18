
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    LL3ToRobot = 
        //LL3 Pose on Robot (x = 17_in, y = -14.5_in, YAW = 20_deg)
        frc::Transform3d(frc::Translation3d(17_in, -14.5_in, 0_in),
            frc::Rotation3d(0_deg, 0_deg, 20_deg));
            frc::Rotation3d Rot3d(0_deg, 0_rad, -20_deg);
            frc::SmartDashboard::PutNumber("Rot3d", Rot3d.Angle().value());
        //LL4 Pose on Robot (x = 16_in, y = 12_in, YAW = -20_deg)
    // LL4ToRobot = 
    //     frc::Transform3d(frc::Translation3d(17_in, 12_in, 0_in),
    //         frc::Rotation3d(0_deg, 0_rad, -20_deg));
    m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, LL3ToRobot);
}

//updates local variables related to the LL3 and LL4 results
void CameraSubsystem::updateData() {
    resultLL3 = limeLight3Camera.GetLatestResult();
    if (resultLL3.HasTargets()) {
        bestTarget = resultLL3.GetBestTarget();
        LL3toTarget = bestTarget.GetBestCameraToTarget();
        //robotPose = robotPose.TransformBy(rawTransformation).TransformBy(LL3ToRobot.Inverse());
        frc::Pose3d robotInvPose = originPose.TransformBy(LL3toTarget).TransformBy(LL3ToRobot.Inverse());
        frc::Pose3d robotPose = originPose.TransformBy(LL3toTarget).TransformBy(LL3ToRobot);
        std::optional<photon::EstimatedRobotPose> estimatedPose = m_poseEstimator->Update(resultLL3);
        
        frc::Pose3d LL3toTargetPose = originPose.TransformBy(LL3toTarget);
        frc::Pose3d robotPosetoLL3 = originPose.TransformBy(LL3ToRobot.Inverse());
        frc::Pose3d resultRobotPose = robotPosetoLL3.RelativeTo(LL3toTargetPose);
        
        frc::SmartDashboard::PutNumber("resultRobotPose X", units::inch_t(resultRobotPose.X()).value());
        frc::SmartDashboard::PutNumber("resultRobotPose Y", units::inch_t(resultRobotPose.Y()).value());


        // X Distance is forwards and backwards, forwards being positive
        // Y Distance is left and right, left being positive
        frc::SmartDashboard::PutNumber("LL3 X Distance", units::inch_t(robotPose.X()).value());
        frc::SmartDashboard::PutNumber("LL3 Y Distance", units::inch_t(robotPose.Y()).value());
        frc::SmartDashboard::PutNumber("LL3 X Inverse Distance", units::inch_t(robotInvPose.X()).value());
        frc::SmartDashboard::PutNumber("LL3 Y Inverse Distance", units::inch_t(robotInvPose.Y()).value());
        frc::SmartDashboard::PutNumber("LL3 Raw X Distance", units::inch_t(LL3toTarget.X()).value());
        frc::SmartDashboard::PutNumber("LL3 Raw Y Distance", units::inch_t(LL3toTarget.Y()).value());
        //frc::SmartDashboard::PutNumber("PoseEstimate X", units::inch_t(m_poseEstimator.get()->().X()).value());



        if (estimatedPose){
            m_drivetrain->AddVisionMeasurement(estimatedPose->estimatedPose.ToPose2d(),ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose->timestamp));
        }
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

double CameraSubsystem::getRotZ() {
    if (visibleTargets()) {
        return LL3toTarget.Rotation().Z().value();
    } else {
        return 0;
    }
};

std::optional<int> CameraSubsystem::lGetTargetTagId() {
    if (visibleTargets()) {
        return LimelightHelpers::getFiducialID(kLimelight4);
    } else {
        return std::nullopt;
    }
}

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    return resultLL3.HasTargets();
}

// bool CameraSubsystem::lVisibleTargets() {
//     return LimelightHelpers::getTV(kLimelight4);
// }

// meters
double CameraSubsystem::getDistance() {
    if (resultLL3.HasTargets()) {
        return units::inch_t(sqrt(pow(getStrafeTransformation().value(), 2) + pow(getForwardTransformation().value(), 2))).value();
    } else {
        return 0;
    }
}

units::meter_t CameraSubsystem::getStrafeTransformation() {
    return LL3toTarget.Y();
}

// units::meter_t CameraSubsystem::lGetStrafeTransformation() {
//     return lTransformation.Y();
// }

units::meter_t CameraSubsystem::getForwardTransformation() {
    return LL3toTarget.X();
}

// units::meter_t CameraSubsystem::lGetForwardTransformation() {
//     return lTransformation.X();
// }

void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Camera Periodic", "Running");

    updateData();

    frc::SmartDashboard::PutNumber("Robot X Position", m_drivetrain->GetPose().X().value());
    frc::SmartDashboard::PutNumber("Robot Y Position", m_drivetrain->GetPose().Y().value());
    frc::SmartDashboard::PutNumber("Robot Rotation", m_drivetrain->GetPose().Rotation().Degrees().value());
}
