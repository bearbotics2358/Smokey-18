#include "subsystems/vision/VisionIOPhotonVision.h"

#include <set>

VisionIOPhotonVision::VisionIOPhotonVision(std::string name, frc::Transform3d robotToCamera):
    m_camera(name),
    m_robotToCamera(robotToCamera) {
}

void VisionIOPhotonVision::updateInputs(VisionIOInputs& inputs) {

    // @todo Do we need logic to determine if the camera is connected? Java PhotonCamera has an IsConnected function. C++ does not.
    inputs.connected = true;

    std::set<int16_t> tagIds;
    std::vector<PoseObservation> poseObservations;

    for (photon::PhotonPipelineResult result : m_camera.GetAllUnreadResults()) {
        // Update latest target observation
        if (result.HasTargets()) {
            inputs.latestTargetObservation =
                TargetObservation(
                    frc::Rotation2d(units::degree_t(result.GetBestTarget().GetYaw())),
                    frc::Rotation2d(units::degree_t(result.GetBestTarget().GetPitch())));
        } else {
            inputs.latestTargetObservation = TargetObservation(frc::Rotation2d(), frc::Rotation2d());
        }

        // Add pose observation
        if (result.MultiTagResult()) {
            photon::MultiTargetPNPResult multitagResult = result.MultiTagResult().value();

            // Calclate robot pose
            frc::Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            frc::Transform3d fieldToRobot = fieldToCamera + m_robotToCamera.Inverse();
            frc::Pose3d robotPose = frc::Pose3d(fieldToRobot.Translation(), fieldToRobot.Rotation());

            // Calculate average tag distance
            units::meter_t totalTagDistance = 0.0_m;
            for (photon::PhotonTrackedTarget target : result.GetTargets()) {
                totalTagDistance += target.GetBestCameraToTarget().Translation().Norm();
            }

            tagIds.insert(multitagResult.fiducialIDsUsed.begin(), multitagResult.fiducialIDsUsed.end());

            poseObservations.push_back(
                PoseObservation(
                    result.GetTimestamp(),
                    robotPose,
                    multitagResult.estimatedPose.ambiguity,
                    multitagResult.fiducialIDsUsed.size(),
                    totalTagDistance / result.GetTargets().size(),
                    PoseObservationType::PHOTONVISION
                )
            );
        } else if (result.GetTargets().empty() == false) {
            // Single tag result
            photon::PhotonTrackedTarget target = result.GetTargets().front();

            std::optional<frc::Pose3d> tagPose = m_aprilTagFieldLayout.GetTagPose(target.GetFiducialId());

            if (tagPose) {
                frc::Transform3d fieldToTarget = frc::Transform3d(tagPose.value().Translation(), tagPose.value().Rotation());
                frc::Transform3d cameraToTarget = target.GetBestCameraToTarget();
                frc::Transform3d fieldToCamera = fieldToTarget + cameraToTarget.Inverse();
                frc::Transform3d fieldToRobot = fieldToCamera + m_robotToCamera.Inverse();
                frc::Pose3d robotPose = frc::Pose3d(fieldToRobot.Translation(), fieldToRobot.Rotation());

                tagIds.insert(target.GetFiducialId());

                poseObservations.push_back(
                    PoseObservation(
                        result.GetTimestamp(),
                        robotPose,
                        target.GetPoseAmbiguity(),
                        1,
                        cameraToTarget.Translation().Norm(),
                        PoseObservationType::PHOTONVISION
                    )
                );
            }
        }
    }

    // Delete all the old pose observations
    inputs.poseObservations.clear();

    // Pre-allocate enough space for all the observations to avoid allocating new memory one observation at a time
    inputs.poseObservations.reserve(poseObservations.size());
    for (PoseObservation& observation : poseObservations) {
        inputs.poseObservations.push_back(observation);
    }

    // Delete all the old tag IDs
    inputs.tagIds.clear();

    // Pre-allocate enough space for all the tag IDs to avoid allocating new memory one tag at a time
    inputs.tagIds.reserve(tagIds.size());

    for (int16_t id : tagIds) {
        inputs.tagIds.push_back(id);
    }
}