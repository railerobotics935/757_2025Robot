#include "Constants.h"
#include "commands/autocommands/ResetHome.h"

ResetHome::ResetHome(ElevatorSubsystem* elevator, CoralPitchSubsystem* coralPitch) : m_elevator{elevator}, m_coralPitch {coralPitch} {
  AddRequirements(m_elevator);
  AddRequirements(m_coralPitch);
}

void ResetHome::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ResetHome Initialized\r\n";
#endif

  m_elevator->GoToSetPoint(0.0);
  m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kMinimumAngle);


}

void ResetHome::End(bool interrupted) {

}