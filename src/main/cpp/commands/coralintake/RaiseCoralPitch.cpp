
#include "Constants.h"
#include "commands/coralintake/RaiseCoralPitch.h"

RaiseCoralPitch::RaiseCoralPitch(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {


  AddRequirements(m_coralPitch);
}

void RaiseCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
currentCoralAngle = m_coralPitch->GetCoralIntakeAngle();
}

void RaiseCoralPitch::Execute() {
  currentCoralAngle += 0.004;
  m_coralPitch->SetCoralIntakeAngle(currentCoralAngle);
}

void RaiseCoralPitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}