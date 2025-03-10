
#include "commands/coralintake/SimpleCoralOuttake.h"


SimpleCoralOuttake::SimpleCoralOuttake(CoralIntakeSubsystem* intake) {
  // Initilize local copys of pointers
  m_coralIntake = intake;

  // Add reqierments for the command
  AddRequirements(m_coralIntake);
}

void SimpleCoralOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(-1.0);
}

void SimpleCoralOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
}