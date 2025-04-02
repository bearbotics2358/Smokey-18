/**
 * @file CameraSubsystem.h
 */

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"
#include "photon/PhotonPoseEstimator.h"

#include "subsystems/CommandSwerveDrivetrain.h"

/**
 * @brief This subsystem uses PhotonVision and a Limelight 3 to recognize AprilTags.
 */
class CameraSubsystem : public frc2::SubsystemBase {
 public:
  CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain);

  /**
   * @brief This function parses camera data.
   * @details This updates local variables, such as the current best AprilTag and
   * the transformation to it. This function __must__ be called periodically to
   * ensure accurate data.
   */
  void updateData();

  /**
   * @brief Returns whether AprilTag(s) exist.
   */
  bool visibleTargets();

  /**
   * @brief Returns the Y **robot oriented** distance from the camera to the best
   * AprilTag.
   * @return The Y distance in meters.
   */
  units::meter_t getStrafeTransformation();

  /**
   * @brief Returns the X **robot oriented** distance from the camera to the best
   * AprilTag.
   * @return The X distance in meters.
   */
  units::meter_t getForwardTransformation();

  /**
   * @brief Returns the straight line distance from the camera to the AprilTag.
   * AprilTag.
   * @return The distance in meters.
   */
  double getDistance();

  /**
   * @brief Returns the Z rotation needed to orient the camera to the AprilTag.
   * @return The distance in degrees.
   */
  units::degree_t getZRotation();

  /**
   * @brief Returns the rotation needed to orient the camera to the AprilTag.
   * @return The rotation is a `frc::Rotation2d`.
   */
  frc::Rotation2d GetRotation2d();

  /**
   * @brief Returns the ID of the best AprilTag, __if__ it exists.
   */
  std::optional<int> GetTargetTagId();

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  photon::PhotonPipelineResult result;
  photon::PhotonTrackedTarget bestTarget;
  frc::Transform3d transformation;

  frc::AprilTagFieldLayout aprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);
  #define CAMERA_NAME "limelight3"
  photon::PhotonCamera limelightCamera{CAMERA_NAME};
  std::unique_ptr<photon::PhotonPoseEstimator> m_poseEstimator;

  subsystems::CommandSwerveDrivetrain* m_drivetrain;

};