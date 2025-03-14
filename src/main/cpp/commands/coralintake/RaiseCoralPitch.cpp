
#include "Constants.h"
#include "commands/coralintake/RaiseCoralPitch.h"

RaiseCoralPitch::RaiseCoralPitch(CoralIntakeSubsystem *coralintake) : m_coralIntake{coralintake} {

  m_coralIntake = coralintake;

  AddRequirements(m_coralIntake);
}

void RaiseCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_coralIntake->SetCoralPitchPower(-0.4);
  //m_coralIntake->SetCoralIntakeAngle(0.3);
}

void RaiseCoralPitch::End(bool interrupted) {
  m_coralIntake->SetCoralPitchPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}