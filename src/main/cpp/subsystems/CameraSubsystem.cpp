
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    LL3ToRobot = 
        frc::Transform3d(frc::Translation3d(16_in, -12_in, 8.5_in),
            frc::Rotation3d(0_deg, 0_deg, 21_deg));
    m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, LL3ToRobot);
}

//updates local variables related to the LL3 and LL4 results
void CameraSubsystem::updateData() {
    resultLL3 = limeLight3Camera.GetLatestResult();
    if (resultLL3.HasTargets()) {
        bestTarget = resultLL3.GetBestTarget();
        LL3toTarget = bestTarget.GetBestCameraToTarget();
        std::optional<photon::EstimatedRobotPose> estimatedPose = m_poseEstimator->Update(resultLL3);
        
        frc::Pose3d LL3toTargetPose = originPose.TransformBy(LL3toTarget);
        frc::Pose3d robotPosetoLL3 = originPose.TransformBy(LL3ToRobot.Inverse());
        LL3ResultRobotPose = robotPosetoLL3.RelativeTo(LL3toTargetPose);
        if (estimatedPose){
            m_drivetrain->AddVisionMeasurement(estimatedPose->estimatedPose.ToPose2d(),ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose->timestamp));
        }
    } else {
        frc::SmartDashboard::PutBoolean("Has Targets", false);
    }


    if (LimelightHelpers::getTV(kLimelight4)) {
        std::vector<double> rawTargetPoseRobotSpace = LimelightHelpers::getTargetPose_RobotSpace(kLimelight4);

        //Limelight software outputs the forward distance as Z, strafe distance as X, and vertical distance as Y.
        // LL4toTarget converts the Limelight outputs from Z(2) to X, X(0) to Y, and Y(1) to Z
        LL4toTarget = frc::Transform3d(units::inch_t(units::meter_t(rawTargetPoseRobotSpace.at(2))), 
                                        units::inch_t(units::meter_t(rawTargetPoseRobotSpace.at(0))),
                                        units::inch_t(units::meter_t(rawTargetPoseRobotSpace.at(1))),
                                        frc::Rotation3d(units::degree_t(rawTargetPoseRobotSpace.at(3)),
                                                        units::degree_t(rawTargetPoseRobotSpace.at(4)),
                                                        units::degree_t(rawTargetPoseRobotSpace.at(5))) 
                                        );


        LimelightHelpers::PoseEstimate lEstimatedPose = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelight4);
        m_drivetrain->AddVisionMeasurement(lEstimatedPose.pose, ctre::phoenix6::utils::FPGAToCurrentTime(lEstimatedPose.timestampSeconds));
    };
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

units::degree_t CameraSubsystem::getZRotation() {
    if (lVisibleTargets() && visibleTargets()) {
        LL4toTarget.Rotation().Z();
    } else if (lVisibleTargets()) {
        LL4toTarget.Rotation().Z();
    } else if (visibleTargets()) {
        LL3ResultRobotPose.Rotation().Z();
    };
}

//Returns true if targets are visible to either or both cameras. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    return resultLL3.HasTargets();
}

bool CameraSubsystem::lVisibleTargets() {
    return LimelightHelpers::getTV();
}


// meters
double CameraSubsystem::getDistance() {
    if (visibleTargets()) {
        return units::inch_t(sqrt(pow(getStrafeTransformation().value(), 2) + pow(getForwardTransformation().value(), 2))).value();
    } else {
        return 0;
    }
}

units::meter_t CameraSubsystem::getStrafeTransformation() {
    if (lVisibleTargets() && visibleTargets()) {
        return ((-units::inch_t((units::meter_t(LL4toTarget.Y()) / 4) * 3) + units::inch_t((LL3ResultRobotPose.Y())/ 4)));
    } else if (lVisibleTargets()) {
        return -units::inch_t(units::meter_t(LL4toTarget.Y()));
    } else if (visibleTargets()) {
        return units::inch_t(LL3ResultRobotPose.Y());
    };
}


units::meter_t CameraSubsystem::getForwardTransformation() {
    if (lVisibleTargets() && visibleTargets()) {
        return ((units::inch_t((units::meter_t(LL4toTarget.X()) / 4) * 3) + units::inch_t((LL3ResultRobotPose.X())/ 4)));
    } else if (lVisibleTargets()) {
        return units::inch_t(units::meter_t(LL4toTarget.X()));
    } else if (visibleTargets()) {
        return units::inch_t(LL3ResultRobotPose.X());
    };
}


void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Camera Periodic", "Running");

    updateData();

    frc::SmartDashboard::PutNumber("Robot X Position", m_drivetrain->GetPose().X().value());
    frc::SmartDashboard::PutNumber("Robot Y Position", m_drivetrain->GetPose().Y().value());
    frc::SmartDashboard::PutNumber("Robot Rotation", m_drivetrain->GetPose().Rotation().Degrees().value());
}
