
#include <subsystems/CameraSubsystem.h>

CameraSubsystem::CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain) {
    m_drivetrain = drivetrain;
    robotToCam = 
        frc::Transform3d(frc::Translation3d(12_in, 34_in, 0_in),
            frc::Rotation3d(-45_deg, 230.5_rad, 135.3_rad));
    m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
}

//updates local variables related to the limelight result
void CameraSubsystem::updateData() {
    if (lVisibleTargets()) {
        std::vector<double> lBotPose = nt::NetworkTableInstance::GetDefault().GetTable(kLimelight4)->GetNumberArray("botpose",std::vector<double>(6));
        frc::SmartDashboard::PutNumber("lTag Dist", units::inch_t(lBotPose[9]).value());
        frc::SmartDashboard::PutNumberArray("DIS ARRAY", lBotPose);

        std::vector<double> lTargetPose = LimelightHelpers::getBotpose_TargetSpace(kLimelight4);

        lTransformation = frc::Transform3d(units::meter_t(lBotPose[9]),
                                            units::meter_t(lTargetPose.at(0)),
                                            units::meter_t(lTargetPose.at(2)),
                                            frc::Rotation3d(units::angle::radian_t(lTargetPose.at(5)),
                                                            units::angle::radian_t(lTargetPose.at(4)),
                                                            units::angle::radian_t(lTargetPose.at(3))));

        LimelightHelpers::PoseEstimate estimatedPose = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelight4);
        m_drivetrain->AddVisionMeasurement(estimatedPose.pose, ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose.timestampSeconds));

        frc::SmartDashboard::PutNumber("lStrafe Distance", units::inch_t(lGetStrafeTransformation()).value());
        frc::SmartDashboard::PutNumber("lForward Distance", units::inch_t(lGetForwardTransformation()).value());
        frc::SmartDashboard::PutNumber("lForward Distance Mega", m_drivetrain->GetPose().X().value() - 14.2);
        frc::SmartDashboard::PutNumber("LNEW DIST", ((sqrt(pow(lBotPose[9], 2)) - pow(0.305, 2)) + 0.43));
        frc::SmartDashboard::PutNumber("IDONTLIKETHIS", (sqrt((pow(getDistance(), 2) + pow(0.50, 2)) - (2 * getDistance() * 0.50 * cosf(90 + getRotZ())))));
        frc::SmartDashboard::PutNumber("IDONTLIKETHIS2", (sqrt((pow(lBotPose[9], 2) + pow(0.50, 2)) - (2 * lBotPose[9] * 0.50 * cosf(90 + lBotPose[6])))));
    }

    frc::SmartDashboard::PutBoolean("lHas Targets", lVisibleTargets());

    result = limelightCamera.GetLatestResult();
    if (result.HasTargets()) {
        bestTarget = result.GetBestTarget();
        transformation = bestTarget.GetBestCameraToTarget();
        frc::Translation3d robotTrans = robotToCam.Translation() + transformation.Translation();
        frc::Rotation3d robotRot = robotToCam.Rotation() + transformation.Rotation();
        frc::Pose3d robotPose = frc::Pose3d(robotTrans, robotRot);

        // frc::SmartDashboard::PutNumber("NEW Strafe Distance", );
        // frc::SmartDashboard::PutNumber("NEW Forward Distance", );

        std::optional<photon::EstimatedRobotPose> estimatedPose = m_poseEstimator->Update(result);
        if (estimatedPose){
            m_drivetrain->AddVisionMeasurement(estimatedPose->estimatedPose.ToPose2d(),ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose->timestamp));
        }

        
        frc::SmartDashboard::PutBoolean("Has Targets", true);

        frc::SmartDashboard::PutNumber("ZRot", units::degree_t(getRotZ()).value());
        frc::SmartDashboard::PutNumber("Rotation", (transformation.Rotation().Z().value() / fabs((transformation.Rotation().Z()).value())) * (180 - fabs(units::degree_t(transformation.Rotation().Z()).value())));
        frc::SmartDashboard::PutNumber("DISTANCE ANGLE", units::degree_t(bestTarget.GetYaw()).value());
        frc::SmartDashboard::PutNumber("Strafe Distance", units::inch_t(getStrafeTransformation()).value());
        frc::SmartDashboard::PutNumber("Forward Distance", units::inch_t(getForwardTransformation()).value());
        frc::SmartDashboard::PutNumber("Dist", units::inch_t(getDistance()).value());
        frc::SmartDashboard::PutNumber("NEW DIST", ((sqrt(pow(getDistance(), 2)) - pow(0.305, 2)) + 0.43));
        //frc::SmartDashboard::PutNumber("NEW DIST", sqrt(pow(getDistance(), 2) - pow(.305, 2)) + 0.43);
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
        return (transformation.Rotation().Z().value() / fabs((transformation.Rotation().Z()).value())) * 
        (180 - fabs(units::degree_t(transformation.Rotation().Z()).value()));
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
    return result.HasTargets();
}

bool CameraSubsystem::lVisibleTargets() {
    return LimelightHelpers::getTV(kLimelight4);
}

// meters
double CameraSubsystem::getDistance() {
    if (result.HasTargets()) {
        return units::inch_t(sqrt(pow(getStrafeTransformation().value(), 2) + pow(getForwardTransformation().value(), 2))).value();
    } else {
        return 0;
    }
}

units::meter_t CameraSubsystem::getStrafeTransformation() {
    return transformation.Y();
}

units::meter_t CameraSubsystem::lGetStrafeTransformation() {
    return lTransformation.Y();
}

units::meter_t CameraSubsystem::getForwardTransformation() {
    return transformation.X();
}

units::meter_t CameraSubsystem::lGetForwardTransformation() {
    return lTransformation.X();
}

void CameraSubsystem::Periodic() {
    frc::SmartDashboard::PutString("Camera Periodic", "Running");

    updateData();

    frc::SmartDashboard::PutNumber("Robot X Position", m_drivetrain->GetPose().X().value());
    frc::SmartDashboard::PutNumber("Robot Y Position", m_drivetrain->GetPose().Y().value());
    frc::SmartDashboard::PutNumber("Robot Rotation", m_drivetrain->GetPose().Rotation().Degrees().value());
}
