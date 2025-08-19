#include "subsystems/vision/VisionIOLimelight.h"

#include <set>
#include <frc/RobotController.h>
#include <networktables/NetworkTableInstance.h>

VisionIOLimelight::VisionIOLimelight(std::string name /*, some way to get the rotation*/) {
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable(name);

    // @todo rotationSupplier assignment goes here

    m_orientationPublisher = table->GetDoubleArrayTopic("robot_orientation_set").Publish();
    m_latencySubscriber = table->GetDoubleTopic("tl").Subscribe(0.0);
    m_txSubscriber = table->GetDoubleTopic("ty").Subscribe(0.0);
    m_megatag1Subscriber = table->GetDoubleArrayTopic("botpose_wpiblue").Subscribe(std::array<double, 0>());
    m_megatag2Subscriber = table->GetDoubleArrayTopic("botpost_orb_wpiblue").Subscribe(std::array<double, 0>());
}

void VisionIOLimelight::updateInputs(VisionIOInputs& inputs) {
    static const int kMillisecondsPerSecond = 1000;

    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = ((frc::RobotController::GetFPGATime() - m_latencySubscriber.GetLastChange()) / kMillisecondsPerSecond) < 250;

    inputs.latestTargetObservation =
        TargetObservation(frc::Rotation2d(units::degree_t(m_txSubscriber.Get())),
                          frc::Rotation2d(units::degree_t(m_tySubscriber.Get())));

    // Update orientation for MegaTag 2

    // @todo replace the first value with the actual orientation of the robot
    std::array<double, 6> orientationArray{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_orientationPublisher.Set(orientationArray);

    // This increases network traffic but it is recommended by Limelight
    nt::NetworkTableInstance::GetDefault().Flush();

    // Read new pose observations from NetworkTables
    std::set<int16_t> tagIds;
    std::vector<PoseObservation> poseObservations;

    for (auto rawSample : m_megatag1Subscriber.ReadQueue()) {
        if (rawSample.value.size() == 0) {
            continue;
        }

        // @todo Figure out what these magic numbers are. These were taken from the Vision Template code.
        for (size_t i = 11; i < rawSample.value.size(); i += 7) {
            tagIds.insert(rawSample.value.at(i));
        }

        poseObservations.push_back(
            PoseObservation(
                // Timestamp, based on server timestamp of publish and latency
                units::second_t(rawSample.time * 1.0e-6 - rawSample.value.at(6) * 1.0e-3),
                parsePose(rawSample.value),

                // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                rawSample.value.size() >= 18 ? rawSample.value.at(17) : 0.0,

                // Tag count
                rawSample.value.at(7),

                // Average tag distance
                units::meter_t(rawSample.value.at(9)),

                // Observation type
                PoseObservationType::MEGATAG_1
            )
        );
    }

    for (auto rawSample : m_megatag2Subscriber.ReadQueue()) {
        if (rawSample.value.size() == 0) {
            continue;
        }

        // @todo Figure out what these magic numbers are. These were taken from the Vision Template code.
        for (size_t i = 11; i < rawSample.value.size(); i += 7) {
            tagIds.insert(rawSample.value.at(i));
        }

        poseObservations.push_back(
            PoseObservation(
                // Timestamp, based on server timestamp of publish and latency
                units::second_t(rawSample.time * 1.0e-6 - rawSample.value.at(6) * 1.0e-3),

                // 3D pose estimate
                parsePose(rawSample.value),

                // Ambiguity, zeroed because the pose is already disambiguated
                0.0,

                // Tag count
                rawSample.value.at(7),

                // Average tag distance
                units::meter_t(rawSample.value.at(9)),

                // Observation type
                PoseObservationType::MEGATAG_2
            )
        );
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