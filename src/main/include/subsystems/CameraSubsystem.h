#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"
#include "photon/PhotonPoseEstimator.h"

#include "subsystems/CommandSwerveDrivetrain.h"

class CameraSubsystem : public frc2::SubsystemBase {
 public:
  CameraSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain);

  void updateData();
  bool visibleTargets();
  units::meter_t getStrafeTransformation();
  units::meter_t getForwardTransformation();
  double getDistance();
  units::degree_t getZRotation();

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