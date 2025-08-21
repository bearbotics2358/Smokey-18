#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"
#include "photon/PhotonPoseEstimator.h"

#include "subsystems/LimelightHelpers.h"

#include "subsystems/CommandSwerveDrivetrain.h"

class CameraSubsystem : public frc2::SubsystemBase {
 public:
  CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain);

  void updateData();
  bool visibleTargets();
  bool lVisibleTargets();
  units::meter_t getStrafeTransformation();
  units::meter_t lGetStrafeTransformation();
  units::meter_t getForwardTransformation();
  units::meter_t lGetForwardTransformation();
  double getDistance();
  units::degree_t getZRotation();
  frc::Rotation2d GetRotation2d();
  double getRotZ();

  std::optional<int> GetTargetTagId();
  std::optional<int> lGetTargetTagId();

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  photon::PhotonPipelineResult resultLL3;
  photon::PhotonTrackedTarget bestTarget;
  frc::Transform3d LL3toTarget;
  frc::Transform3d LL3ToRobot;
  frc::Transform3d LL4ToRobot;
  frc::Pose3d originPose;

  frc::Transform3d lTransformation;

  frc::AprilTagFieldLayout aprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
  #define CAMERA_NAME "limelight3"
  photon::PhotonCamera limeLight3Camera{CAMERA_NAME};
  std::unique_ptr<photon::PhotonPoseEstimator> m_poseEstimator;

  const std::string kLimelight4 = "limelight";

  subsystems::CommandSwerveDrivetrain* m_drivetrain;

};