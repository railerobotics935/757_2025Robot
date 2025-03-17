
#include "Constants.h"
#include "commands/coralintake/StopCoralPitch.h"

StopCoralPitch::StopCoralPitch(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {


  AddRequirements(m_coralPitch);
}

void StopCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
// Stops the intake from snapping back to setpoint when enabled
//  m_coralPitch->SetCoralIntakeAngle(m_coralPitch->GetCoralIntakeAngle());
}

void StopCoralPitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}