// commands/ChangeLEDs.h

#ifndef COMMANDS_CHANGELEDS_H
#define COMMANDS_CHANGELEDS_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/LED.h"
#include "Constants.h"

class ChangeLEDs : public frc2::CommandHelper<frc2::Command, ChangeLEDs> {
public:
    explicit ChangeLEDs(LED* ledSubsystem, ArduinoConstants::RIO_MESSAGES ledState);

    void Initialize() override;

private:
    LED* m_LED;
    ArduinoConstants::RIO_MESSAGES m_ledState;
};

#endif // COMMANDS_CHANGELEDS_H
