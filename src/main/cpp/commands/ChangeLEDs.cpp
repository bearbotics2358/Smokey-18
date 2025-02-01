#include <commands/ChangeLEDs.h>

ChangeLEDs::ChangeLEDs(LED* ledSubsystem, ArduinoConstants::RIO_MESSAGES ledState) {
    AddRequirements(ledSubsystem);
    m_ledState = ledState;
}

void ChangeLEDs::Initialize() {
    (m_LED->m_LEDArray[static_cast<int>(m_ledState)])();
}