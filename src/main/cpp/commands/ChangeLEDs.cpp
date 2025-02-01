#include <commands/ChangeLEDs.h>

ChangeLEDs::ChangeLEDs(LED* ledSubsystem, RIO_msgs_enum ledState) {
    AddRequirements(ledSubsystem);
    m_ledState = ledState;
}

void ChangeLEDs::Initialize() {
    (m_LED->m_LEDArray[static_cast<int>(m_ledState)])();
}

void ChangeLEDs::ChangeLEDState(RIO_msgs_enum ledState) {
    (m_LED->m_LEDArray[static_cast<int>(ledState)])();
}