
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
    std::map<int, TagObservation> ll3Tags = GetRelevantTagObservationsPV(resultLL3);

    if (resultLL3.HasTargets()) {
        std::optional<photon::EstimatedRobotPose> estimatedPose = m_poseEstimator->Update(resultLL3);
        if (estimatedPose){
            m_drivetrain->AddVisionMeasurement(estimatedPose->estimatedPose.ToPose2d(),ctre::phoenix6::utils::FPGAToCurrentTime(estimatedPose->timestamp));
        }

        // ---------------------------------------------------------------------------------------------------------------------
        // @todo After implementing the mechanism to read the list of TagObservations, we probably don't need this bit of code
        bestTarget = resultLL3.GetBestTarget();
        LL3toTarget = bestTarget.GetBestCameraToTarget();

        frc::Pose3d LL3toTargetPose = originPose.TransformBy(LL3toTarget);
        frc::Pose3d robotPosetoLL3 = originPose.TransformBy(LL3ToRobot.Inverse());
        LL3ResultRobotPose = robotPosetoLL3.RelativeTo(LL3toTargetPose);
        // ---------------------------------------------------------------------------------------------------------------------
    } else {
        frc::SmartDashboard::PutBoolean("Has Targets", false);
    }


    std::map<int, TagObservation> ll4Tags = GetRelevantTagObservationsLL4();
    if (LimelightHelpers::getTV(kLimelight4)) {

        LimelightHelpers::PoseEstimate lEstimatedPose = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelight4);
        m_drivetrain->AddVisionMeasurement(lEstimatedPose.pose, ctre::phoenix6::utils::FPGAToCurrentTime(lEstimatedPose.timestampSeconds));

        // ---------------------------------------------------------------------------------------------------------------------
        // @todo After implementing the mechanism to read the list of TagObservations, we probably don't need this bit of code
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
        // ---------------------------------------------------------------------------------------------------------------------
    };

    PopulateFinalTargets(ll3Tags, ll4Tags);
}

std::optional<int> CameraSubsystem::GetTargetTagId() {

    // ---------------------------------------------------------------------------------------------------------------------
    // @todo Replace this block with logic that determines the Best Target from the saved list of targets
    if (visibleTargets()) {
        return bestTarget.GetFiducialId();
    } else {
        return std::nullopt;
    }
    // ---------------------------------------------------------------------------------------------------------------------
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

std::map<int, TagObservation> CameraSubsystem::GetRelevantTagObservationsPV(photon::PhotonPipelineResult photonResult) {
    std::map<int, TagObservation> tags;

    for (photon::PhotonTrackedTarget target : photonResult.GetTargets()) {
        if (IsTagWeCareAboutForTargeting(target.GetFiducialId())) {
            int tagId = target.GetFiducialId();
            frc::Transform3d cameraToTarget = target.GetBestCameraToTarget();
            units::meter_t distance = cameraToTarget.Translation().Norm();

            frc::Pose3d cameraToTargetPose = originPose.TransformBy(cameraToTarget);
            frc::Pose3d robotPoseToCamera = originPose.TransformBy(LL3ToRobot.Inverse());
            frc::Translation3d targetToBot = robotPoseToCamera.RelativeTo(cameraToTargetPose).Translation();

            TagObservation observation(tagId, distance, targetToBot, kPhotonVision);

            tags[tagId] = observation;
        }
    }

    return tags;
}

std::map<int, TagObservation> CameraSubsystem::GetRelevantTagObservationsLL4() {
    std::map<int, TagObservation> tags;

    // @todo Implement this function similar to the PhotonVision one where we return the TagObservations for LL4

    return tags;
}

bool CameraSubsystem::IsTagWeCareAboutForTargeting(int tagId) {
    bool result = true;

    // @todo Add functionality here that determines if this is a tag we should care about for targeting
    //       Do we need more parameters other than just the tagId?

    return result;
}

void CameraSubsystem::PopulateFinalTargets(std::map<int, TagObservation>& ll3Observations, std::map<int, TagObservation>& ll4Observations) {

    // Always clean the old observations to avoid stale ones sticking around when a tag is not visible any more
    m_targetList.clear();

    // Using pointers here to avoid copying the lists unnecessarily
    std::map<int, TagObservation>* biggerList;
    std::map<int, TagObservation>* smallerList;

    // The list of observations may not be the same size. Figure out which one is bigger so that is the one we loop through
    if (ll3Observations.size() > ll4Observations.size()) {
        biggerList = &ll3Observations;
        smallerList = &ll4Observations;
    } else {
        biggerList = &ll4Observations;
        smallerList = &ll3Observations;
    }

    for (auto iterator = biggerList->begin(); iterator != biggerList->end(); iterator++) {
        // The iterator contains both the key and value for the item in the map.
        // The key is the tagId and the value is the TagObservation.
        int tagId = iterator->first;
        std::optional<TagObservation> observation1 = iterator->second;
        std::optional<TagObservation> observation2 = std::nullopt;

        if (smallerList->contains(tagId)) {
            observation2 = smallerList->at(tagId);
        }

        if (std::nullopt == observation2) {
            TagObservation mergedObservation(tagId, observation1->distanceToTarget, observation1->targetToBotTranslation, kMerged);
            m_targetList[tagId] = mergedObservation;
        } else {
            frc::Translation3d weighted1 = GetWeightedTranslation3d(observation1.value());
            frc::Translation3d weighted2 = GetWeightedTranslation3d(observation2.value());
            frc::Translation3d merged = (weighted1 + weighted2) / 2;

            units::meter_t distance = merged.Norm();
            TagObservation mergedObservation(tagId, distance, merged, kMerged);

            m_targetList[tagId] = mergedObservation;
        }
    }
}

frc::Translation3d CameraSubsystem::GetWeightedTranslation3d(TagObservation observation) {
    const double kPhotonVisionWeight = 0.25;
    const double kLimelight4Weight = 0.75;

    frc::Translation3d weighted;

    frc::Translation3d translation = observation.targetToBotTranslation;
    if (observation.observerType == kPhotonVision) {
        weighted = translation * kPhotonVisionWeight;
    } else {
        weighted = translation * kLimelight4Weight;
    }

    return weighted;
}