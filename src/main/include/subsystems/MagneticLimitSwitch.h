#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

// Class for a Magnetic Limit Switches.
class LimitSwitch : public frc2::SubsystemBase {
   public:
        LimitSwitch(int port);
        bool GetBeamBroken();
   private:
       frc::DigitalInput m_Input;
};