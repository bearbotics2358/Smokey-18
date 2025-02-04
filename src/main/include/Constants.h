#include <unordered_map>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

namespace VisionConstants {
    // TODO: make the poses correct
    const std::unordered_map<int, frc::Pose2d> aprilTagsToPoses = {
        {6, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {7, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {8, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {9, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {10, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {11, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {17, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {18, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {19, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {20, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {21, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
        {22, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))},
    };
}