#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <subsystems/LED.h>
#include <Protocol2025.h>

class ChangeLEDs : public frc2::CommandHelper<frc2::Command, ChangeLEDs> {
public:
    explicit ChangeLEDs(LED* ledSubsystem, RIO_msgs_enum ledState);

    void Initialize() override;
    void ChangeLEDState(RIO_msgs_enum ledState);
private:
    LED* m_LED;

    RIO_msgs_enum m_ledState;
};
