#pragma once

#include "subsystems/vision/VisionIO.h"

#include <functional>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>

typedef std::function<units::degree_t()> BotHeadingProvider;

/**
 * IO implementation for real Limelight hardware
 */
class VisionIOLimelight : public VisionIO {
public:
    VisionIOLimelight(std::string name, BotHeadingProvider getBotHeadingDegrees);

    void updateInputs(VisionIOInputs& inputs) override;

private:
    BotHeadingProvider m_getBotHeadingDegrees;
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