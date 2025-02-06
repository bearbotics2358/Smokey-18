#include "subsystems/MagneticLimitSwitch.h"

LimitSwitch::LimitSwitch(int port):
a_Input(port) {}


bool LimitSwitch::GetBeamBroken() {
   return a_Input.Get();
};