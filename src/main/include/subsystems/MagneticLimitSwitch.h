#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

// Class for a Magnetic Limit Switchs.

class LimitSwitch : public frc2::SubsystemBase {
   public:
        LimitSwitch(int port);
        bool LimitDetection();
   private:
       frc::DigitalInput a_Input;
};