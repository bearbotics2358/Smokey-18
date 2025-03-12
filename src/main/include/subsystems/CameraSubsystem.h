
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"

class CameraSubsystem : public frc2::SubsystemBase {
 public:
  CameraSubsystem();
  void Periodic() override;

  void UpdateData();
  bool VisibleTargets();

  double GetHorizontalTransformation();
  double GetZRotation();
  double GetYDistance();

  frc::Pose2d GetRobotPosition();

 private:
  photon::PhotonPipelineResult result;
  photon::PhotonTrackedTarget bestTarget;
  frc::Transform3d transformation;

  frc::AprilTagFieldLayout aprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);
  #define CAMERA_NAME "limelight3"
  photon::PhotonCamera limelightCamera{CAMERA_NAME};
};