#include "Constants.h"
#include "commands/algaeintake/StopAlgaeIntake.h"

StopAlgaeIntake::StopAlgaeIntake(AlgaeIntakeSubsystem* algaeintake) : m_algaeIntake{algaeintake} {
  AddRequirements(m_algaeIntake);
}

void StopAlgaeIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopIntake Initialized\r\n";
#endif

  m_algaeIntake->SetAlgaeIntakeMotorPower(0.0);
}

void StopAlgaeIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StopIntake Ended\r\n";
#endif
}