
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    LL3ToRobot = 
        //LL3 Pose on Robot (x = 17_in, y = -14.5_in, YAW = 20_deg)
        frc::Transform3d(frc::Translation3d(16_in, -12_in, 0_in),
            frc::Rotation3d(0_deg, 0_deg, 21_deg));
    LL4ToRobot = 
        frc::Transform3d(frc::Translation3d(16_in, 12_in, 0_in),
            frc::Rotation3d(0_deg, 0_rad, -21_deg));
    m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, LL3ToRobot);
}

//updates local variables related to the LL3 and LL4 results
void CameraSubsystem::updateData() {
    resultLL3 = limeLight3Camera.GetLatestResult();
    if (resultLL3.HasTargets()) {
        bestTarget = resultLL3.GetBestTarget();
        LL3toTarget = bestTarget.GetBestCameraToTarget();
        //robotPose = robotPose.TransformBy(rawTransformation).TransformBy(LL3ToRobot.Inverse());
        std::optional<photon::EstimatedRobotPose> estimatedPose = m_poseEstimator->Update(resultLL3);
        
        frc::Pose3d LL3toTargetPose = originPose.TransformBy(LL3toTarget);
        frc::Pose3d robotPosetoLL3 = originPose.TransformBy(LL3ToRobot.Inverse());
        LL3ResultRobotPose = robotPosetoLL3.RelativeTo(LL3toTargetPose);


        // X Distance is forwards and backwards, forwards being positive
        // Y Distance is left and right, left being positive
        frc::SmartDashboard::PutNumber("LL3 Raw X Distance", units::inch_t(LL3toTarget.X()).value());
        frc::SmartDashboard::PutNumber("LL3 Raw Y Distance", units::inch_t(LL3toTarget.Y()).value());
        frc::SmartDashboard::PutNumber("LL3 X Distance", units::inch_t(robotPosetoLL3.X()).value());
        frc::SmartDashboard::PutNumber("LL3 Y Distance", units::inch_t(robotPosetoLL3.Y()).value());
        frc::SmartDashboard::PutNumber("LL3ResultRobotPose X", units::inch_t(LL3ResultRobotPose.X()).value());
        frc::SmartDashboard::PutNumber("LL3ResultRobotPose Y", units::inch_t(LL3ResultRobotPose.Y()).value());
        //frc::SmartDashboard::PutNumber("Megatag YAW", units::degree(LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2().pose.Rotation()).value());
        //frc::SmartDashboard::PutNumber("PoseEstimate X", units::inch_t(m_poseEstimator.get()->().X()).value());



        if (estimatedPose){
            m_drivetrain->AddVisionMeasurement(estimatedPose->estimatedPose.ToPose2d(),ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose->timestamp));
        }
    } else {
        frc::SmartDashboard::PutBoolean("Has Targets", false);
    }


    if (LimelightHelpers::getTV(kLimelight4)) {
        std::vector<double> LL4Result = nt::NetworkTableInstance::GetDefault().GetTable(kLimelight4)->GetNumberArray("botpose", std::vector<double>(6));
        std::vector<double> LL4TargetPose = LimelightHelpers::getTargetPose_RobotSpace(kLimelight4);

        LL4toTarget = frc::Transform3d(units::inch_t(units::meter_t(LL4Result[16])),
                                        units::inch_t(units::meter_t(LL4TargetPose.at(0))),
                                        units::inch_t(units::meter_t(LL4Result[3])),
                                        frc::Rotation3d(units::angle::degree_t(LL4Result.at(3)),
                                        units::angle::degree_t(LL4Result.at(3)),
                                        units::angle::degree_t(LL4Result.at(5)))
                                    );

        // frc::Pose3d LL4toTargetPose = originPose.TransformBy(LL4toTarget);
        // frc::Pose3d robotPosetoLL4 = originPose.TransformBy(LL4ToRobot.Inverse());
        // frc::Pose3d LL4ResultRobotPose = LL4toTargetPose.RelativeTo(robotPosetoLL4);


        LimelightHelpers::PoseEstimate lEstimatedPose = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelight4);
        m_drivetrain->AddVisionMeasurement(lEstimatedPose.pose, ctre::phoenix6::utils::FPGAToCurrentTime(lEstimatedPose.timestampSeconds));

        // frc::SmartDashboard::PutNumberArray("LL4 Raw Distance", LL4Result);
        // frc::SmartDashboard::PutNumber("LL4 Raw YAW", units::degree_t(LL4toTarget.Rotation().X()).value());
        // frc::SmartDashboard::PutNumber("LL4 Raw X Distance", units::inch_t(units::meter_t(LL4Result[16])).value());
        // frc::SmartDashboard::PutNumber("LL4 X Distance", units::inch_t(LL4toTargetPose.X()).value());
        // frc::SmartDashboard::PutNumber("LL4 X Pose", units::inch_t(robotPosetoLL4.X()).value());
        // frc::SmartDashboard::PutNumber("LL4 Raw Y Distance", -units::inch_t(units::meter_t(LL4toTarget.Y())).value());
        // frc::SmartDashboard::PutNumber("LL4 Y Distance", units::inch_t(LL4toTargetPose.Y()).value());
        // frc::SmartDashboard::PutNumber("LL4 Y Pose", units::inch_t(robotPosetoLL4.Y()).value());
        // frc::SmartDashboard::PutNumber("LL4ResultRobotPose X", units::inch_t(LL4ResultRobotPose.X()).value());
        // frc::SmartDashboard::PutNumber("LL4ResultRobotPose Y", units::inch_t(LL4ResultRobotPose.Y()).value());
    };
        frc::SmartDashboard::PutNumber("Tag X Distance", (units::inch_t(units::meter_t(LL4toTarget.X())).value() + units::inch_t(LL3ResultRobotPose.X()).value()) / 2);
        frc::SmartDashboard::PutNumber("Tag Y Distance", (-units::inch_t(units::meter_t(LL4toTarget.Y())).value() + units::inch_t(LL3ResultRobotPose.Y()).value()) / 2);
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

//Returns true if targets are visible to limelight. Otherwise returns false
bool CameraSubsystem::visibleTargets() {
    return resultLL3.HasTargets();
}

bool CameraSubsystem::lVisibleTargets() {
    return LimelightHelpers::getTV();
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
    return ((-units::inch_t(units::meter_t(LL4toTarget.Y())) + units::inch_t(LL3ResultRobotPose.Y())) / 2);
}

// units::meter_t CameraSubsystem::lGetStrafeTransformation() {
//     return lTransformation.Y();
// }

units::meter_t CameraSubsystem::getForwardTransformation() {
    return ((units::inch_t(units::meter_t(LL4toTarget.X())) + units::inch_t(LL3ResultRobotPose.X())) / 2);
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
