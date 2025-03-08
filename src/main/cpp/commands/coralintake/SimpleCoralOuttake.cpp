
#include "commands/intake/SimpleOuttake.h"


SimpleOuttake::SimpleOuttake(IntakeSubsystem* intake) {
  // Initilize local copys of pointers
  m_intake = intake;

  // Add reqierments for the command
  AddRequirements(m_intake);
}

void SimpleOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(-1.0);
}

void SimpleOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
}