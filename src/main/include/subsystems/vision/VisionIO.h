#pragma once

#include <vector>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>

class VisionIO {
public:
    /** Represents the angle to a simple target, not used for pose estimation. */
    typedef struct TargetObservation {
        frc::Rotation2d tx;
        frc::Rotation2d ty;
    };

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    };

    /** Represents a robot pose sample used for pose estimation. */
    typedef struct PoseObservation {
        units::time::second_t timestamp;
        frc::Pose3d pose;
        double ambiguity;
        int tagCount;
        units::meter_t averageTagDistance;
        PoseObservationType type;
    };

    typedef struct VisionIOInputs {
        bool connected = false;
        TargetObservation latestTargetObservation;
        std::vector<PoseObservation> poseObservations;
        std::vector<int16_t> tagIds;
    };

    virtual void updateInputs(VisionIOInputs& inputs) = 0;
};