
#include "commands/algaeintake/SimpleAlgaeOuttake.h"


SimpleAlgaeOuttake::SimpleAlgaeOuttake(AlgaeIntakeSubsystem* algaeintake) {
  // Initilize local copys of pointers
  m_algaeIntake = algaeintake;

  // Add reqierments for the command
  AddRequirements(m_algaeIntake);
}

void SimpleAlgaeOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_algaeIntake->SetAlgaeIntakeMotorPower(-1.0);
}

void SimpleAlgaeOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_algaeIntake->SetAlgaeIntakeMotorPower(0.0);
}