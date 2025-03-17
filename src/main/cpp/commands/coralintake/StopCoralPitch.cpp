
#include "Constants.h"
#include "commands/coralintake/StopCoralPitch.h"

StopCoralPitch::StopCoralPitch(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {

  m_coralPitch = coralpitch;

  AddRequirements(m_coralPitch);
}

void StopCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  //m_coralIntake->SetCoralIntakeAngle(0.0);
}

void StopCoralPitch::End(bool interrupted) {
  //m_coralIntake->SetCoralIntakeAngle(0.0);
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}