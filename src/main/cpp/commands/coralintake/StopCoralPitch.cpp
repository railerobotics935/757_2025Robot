
#include "Constants.h"
#include "commands/coralintake/StopCoralPitch.h"

StopCoralPitch::StopCoralPitch(CoralIntakeSubsystem *coralintake) : m_coralIntake{coralintake} {

  m_coralIntake = coralintake;

  AddRequirements(m_coralIntake);
}

void StopCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_coralIntake->SetCoralPitchPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.3);
}

void StopCoralPitch::End(bool interrupted) {
  m_coralIntake->SetCoralPitchPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}