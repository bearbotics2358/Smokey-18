#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/LED.h"
#include "Constants.h"

class ChangeLEDs : public frc2::CommandHelper<frc2::Command, ChangeLEDs> {
public:
    explicit ChangeLEDs(LED* ledSubsystem, ArduinoConstants::RIO_MESSAGES ledState);

    void Initialize() override;
    LED* m_LED;
private:

    ArduinoConstants::RIO_MESSAGES m_ledState;
};
