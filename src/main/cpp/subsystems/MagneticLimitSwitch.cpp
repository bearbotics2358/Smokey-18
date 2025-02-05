#include "subsystems/MagneticLimitSwitch.h"


LimitSwitch::LimitSwitch(int port):
a_Input(port) {}


bool LimitSwitch::LimitDetection() {
   return a_Input.Get();
};