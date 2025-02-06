#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>

class ThroughBoreEncoder : public frc2::SubsystemBase {
    public:
        ThroughBoreEncoder();
        double GetDistance();
        double GetDirection();
    private:
        // TODO: {0, 1} sets up the DIO ports on 0 and 1. Change accordingly.
        frc::Encoder m_throughBorEncoder{0, 1, false, frc::Encoder::EncodingType::k4X};
};