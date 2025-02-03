#include <commands/ChangeLEDs.h>

ChangeLEDs::ChangeLEDs(LED* ledSubsystem, ArduinoConstants::RIO_MESSAGES ledState) {
    AddRequirements(ledSubsystem);
    m_ledState = ledState;
}

void ChangeLEDs::Initialize() {
    m_LED->SetLEDState(m_ledState);
}