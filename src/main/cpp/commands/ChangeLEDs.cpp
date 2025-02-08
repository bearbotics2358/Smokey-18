#include "commands/ChangeLEDs.h"

ChangeLEDs::ChangeLEDs(LED* ledSubsystem, ArduinoConstants::RIO_MESSAGES ledState):
m_LED(ledSubsystem), m_ledState(ledState)
{
    AddRequirements(ledSubsystem);
}

void ChangeLEDs::Initialize() {
    m_LED->SetLEDState(m_ledState); 
}