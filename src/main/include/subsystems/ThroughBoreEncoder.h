#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>

class ThroughBoreEncoder : public frc2::SubsystemBase {
    public:
        ThroughBoreEncoder(int dioPortA, int dioPortB);
        double GetDistance();
        double GetDirection();
        void ResetEncoder();
    private:
        frc::Encoder m_throughBoreEncoder;
};