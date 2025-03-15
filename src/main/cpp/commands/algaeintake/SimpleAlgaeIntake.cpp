
#include "Constants.h"
#include "commands/algaeintake/SimpleAlgaeIntake.h"

SimpleAlgaeIntake::SimpleAlgaeIntake(AlgaeIntakeSubsystem *algaeintake) : m_algaeIntake{algaeintake} {

  AddRequirements(m_algaeIntake);
}

void SimpleAlgaeIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_algaeIntake->SetAlgaeIntakeMotorPower(0.3);
}


void SimpleAlgaeIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_algaeIntake->SetAlgaeIntakeMotorPower(0.0);
}