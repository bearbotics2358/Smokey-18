#pragma once

#include "subsystems/vision/VisionIO.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <photon/PhotonCamera.h>

/**
 * IO implementation for real PhotonVision hardware
 */
class VisionIOPhotonVision : public VisionIO {
public:
    VisionIOPhotonVision(std::string name, frc::Transform3d robotToCamera);

    void updateInputs(VisionIOInputs& inputs) override;

private:
    photon::PhotonCamera m_camera;
    frc::Transform3d m_robotToCamera;

    frc::AprilTagFieldLayout m_aprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
};