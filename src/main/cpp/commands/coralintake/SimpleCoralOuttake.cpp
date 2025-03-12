
#include "commands/coralintake/SimpleCoralOuttake.h"


SimpleCoralOuttake::SimpleCoralOuttake(CoralIntakeSubsystem* coralintake) : m_coralIntake{coralintake} {
  // Initilize local copys of pointers
  m_coralIntake = coralintake;

  // Add reqierments for the command
  AddRequirements(m_coralIntake);
}

void SimpleCoralOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.5);
}

void SimpleCoralOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
}