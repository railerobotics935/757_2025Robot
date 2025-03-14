#include "Constants.h"
#include "commands/coralintake/StopCoralIntake.h"

StopCoralIntake::StopCoralIntake(CoralIntakeSubsystem* coralintake) : m_coralIntake{coralintake} {
  AddRequirements(m_coralIntake);
}

void StopCoralIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopIntake Initialized\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
}

void StopCoralIntake::End(bool interrupted) {
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "StopIntake Ended\r\n";
#endif
}