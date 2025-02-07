#include "subsystems/MagneticLimitSwitch.h"

LimitSwitch::LimitSwitch(int port):
m_Input(port) {}


bool LimitSwitch::GetBeamBroken() {
   return m_Input.Get();
};