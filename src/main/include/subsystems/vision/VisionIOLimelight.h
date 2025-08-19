#pragma once

#include "subsystems/vision/VisionIO.h"

#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>

/**
 * IO implementation for real Limelight hardware
 */
class VisionIOLimelight : public VisionIO {
public:
    VisionIOLimelight(std::string name /*, add a variable to get rotation in some way */);

    void updateInputs(VisionIOInputs& inputs) override;

private:
    nt::DoubleArrayPublisher m_orientationPublisher;

    nt::DoubleSubscriber m_latencySubscriber;
    nt::DoubleSubscriber m_txSubscriber;
    nt::DoubleSubscriber m_tySubscriber;
    nt::DoubleArraySubscriber m_megatag1Subscriber;
    nt::DoubleArraySubscriber m_megatag2Subscriber;

    static frc::Pose3d parsePose(std::vector<double>& rawLLArray) {
        return frc::Pose3d(
            units::meter_t(rawLLArray.at(0)),
            units::meter_t(rawLLArray.at(1)),
            units::meter_t(rawLLArray.at(2)),
            frc::Rotation3d(
                units::degree_t(rawLLArray.at(3)),
                units::degree_t(rawLLArray.at(4)),
                units::degree_t(rawLLArray.at(5))));
    }
};